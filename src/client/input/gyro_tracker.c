//
// Created by Campbell on 23/07/2024.
//

#define HANDMADE_MATH_IMPLEMENTATION

#include "gyro_tracker.h"

#include <stdint.h>

static const hmm_vec3 UNIT_X = {1, 0, 0};
static const hmm_vec3 UNIT_Y = {0, 1, 0};
static const hmm_vec3 UNIT_Z = {0, 0, 1};

// Reference frames:
//
// ECEF frame (earth-centred, earth-fixed):
//    The earth's reference frame.
//    (technically this isn't an inertial frame since it neglects
//    the earth's rotation, but that's completely negligible
//    compared to the gyro's steady-state error).
//
// Initial frame:
//    The reference frame in which the x,y,z axes matched SDL's definition
//    of the controller's axes at the time when the controller sent us it's
//    first sample.
//    We assume the controller is horizontal (Y aligned with gravity) when
//    it's first sample is received. Of course this isn't true, but it doesn't
//    affect the output (which is the difference between the current and
//    recentre frames) and it gives us a simple rule we can use with the
//    accelerometer to correct for the gyroscope's DC bias.
//    (Not to be confused with inertial frames, which this isn't)
//
// Current frame:
//    The reference frame in which the x,y,z axes align with SDL's definition
//    of them for the current controller position.
//
// Recentre frame:
//    The reference frame in which the x,y,z axes align with SDL's definition
//    of them for the controller position, when the gyroscope was last recentred
//    by Lua (either through the user pressing a reset button, or automatically
//    when performing or stopping some action).
//
// The end goal of this system is to find the difference between the recentre frame
// and the current frame - this represents the rotation of the controller since it
// was last recentred (see GetForwards).

// The rotation that transforms an object from the current frame to the initial frame.
static hmm_quaternion m_currentToInitial = {0, 0, 0, 1};

// The rotation that transforms an object from the initial frame to the recentre frame.
// This is stored inverted (vs being initial-to-recentred) since we'll use it every
// time we read the forwards vector.
static hmm_quaternion m_initialToRecentre = {0, 0, 0, 1};

static hmm_vec3 m_lastAngVel = {0, 0, 0};
static uint64_t m_lastGyroTimestamp = 0;

static uint64_t m_lastAccelerometerTimestamp = 0;


static hmm_vec3 GetMatrixColumn(hmm_mat4 matrix, int column) {
	hmm_vec3 result = {
			// I've double-checked and this column/row ordering is correct.
			matrix.Elements[column][0],
			matrix.Elements[column][1],
			matrix.Elements[column][2],
	};
	return result;
}

void GyroTracker_PushGyroEvent(uint64_t timestampNS, hmm_vec3 angularVelocity) {
	// If there's a big time gap, assume that something interrupted
	// the data and we can't meaningfully integrate over that gap.
	uint64_t deltaNS = timestampNS - m_lastGyroTimestamp;
	m_lastGyroTimestamp = timestampNS;
	if (deltaNS > 500000000) {
		m_lastAngVel = angularVelocity;
		return;
	}

	// Assume the true angular velocity was linearly interpolated between
	// the previous and current values.
	float deltaS = (float) deltaNS / 1.0e9f;

	// This is what the integral of f=a(1-t/T)+b(t/T) works out to, where t
	// is the time since the previous sample and T is the period between
	// that sample (a) and the current sample (b).
	hmm_vec3 averageAngVel = HMM_DivideVec3f(HMM_AddVec3(m_lastAngVel, angularVelocity), 2.f);
	hmm_vec3 integratedEuler = HMM_MultiplyVec3f(averageAngVel, deltaS);

	// Convert the Euler rotation angles to a quaternion
	// TODO is the rotation order here an accuracy problem?
	hmm_quaternion thisUpdateRotation = HMM_QuaternionFromAxisAngle(UNIT_X, integratedEuler.X);
	thisUpdateRotation = HMM_MultiplyQuaternion(thisUpdateRotation,
												HMM_QuaternionFromAxisAngle(UNIT_Y, integratedEuler.Y));
	thisUpdateRotation = HMM_MultiplyQuaternion(thisUpdateRotation,
												HMM_QuaternionFromAxisAngle(UNIT_Z, integratedEuler.Z));

	m_currentToInitial = HMM_MultiplyQuaternion(m_currentToInitial, thisUpdateRotation);

	m_lastAngVel = angularVelocity;
}

void GyroTracker_PushAccelerometerEvent(uint64_t timestampNS, hmm_vec3 acceleration) {
	// If there's a big time gap, assume that something interrupted
	// the data and we can't meaningfully integrate over that gap.
	uint64_t deltaNS = timestampNS - m_lastAccelerometerTimestamp;
	m_lastAccelerometerTimestamp = timestampNS;
	if (deltaNS > 500000000) {
		return;
	}

	float deltaS = (float) deltaNS / 1.0e9f;

	float accelerationMagnitude = HMM_LengthVec3(acceleration);
	if (accelerationMagnitude == 0) {
		return; // Block divide-by-zeros
	}

	// Find the normalised vector representing where the acceleration is, which we assume
	// is where gravity is - accelerations from the player shaking the controller should
	// approximately cancel out.
	// TODO filtering to stop high-frequency noise messing with this.
	hmm_vec3 gravity = HMM_DivideVec3f(acceleration, accelerationMagnitude);

	// Find out where we gravity *should* be, from our definition of the initial reference
	// frame being horizontally aligned.
	// We use the inverse to convert from the initial frame to the current frame.
	// Note that column 1 of the matrix represents the +ve Y axis.
	hmm_quaternion initialToCurrent = HMM_InverseQuaternion(m_currentToInitial);
	hmm_vec3 down = HMM_MultiplyVec3f(GetMatrixColumn(HMM_QuaternionToMat4(initialToCurrent), 1), -1);

	// Find the angle-axis difference between these two vectors.
	hmm_vec3 cross = HMM_Cross(down, gravity);
	float length = HMM_LengthVec3(cross);
	if (length == 0) {
		return; // No drift at all! Block the divide-by-zero.
	}
	float angle = asinf(length);
	hmm_vec3 axis = HMM_DivideVec3f(cross, length);

	// Figure out how much to rotate the state by to correct for the gyro drift
	// This effectively forms an IIR filter.
	float angleToCorrect = angle * deltaS;
	hmm_quaternion correction = HMM_QuaternionFromAxisAngle(axis, angleToCorrect);
	m_currentToInitial = HMM_MultiplyQuaternion(m_currentToInitial, correction);
}

void GyroTracker_Recentre() {
	// Find the player's yaw, relative to the initial frame
	// +ve is CCW when viewed from above
	// Note column 2 represents +ve Z, which points towards the player.
	hmm_vec3 forwards = HMM_MultiplyVec3f(GetMatrixColumn(HMM_QuaternionToMat4(m_currentToInitial), 2), -1);
	float yaw = atan2f(-forwards.X, -forwards.Z);

	// Find the player's pitch, where +ve is above the horizon and -ve is below
	float pitch = asinf(forwards.Y);

	// Find the local->world transform just based on these, ignoring the roll.
	// Note we *have* to include the yaw here if we want to cancel out the pitch - because
	// the plane through which the pitch acts varies depending on the yaw, if we don't
	// include the yaw then weird things happen, which is most noticeable with recentring
	// not putting the camera back to the horizon.
	hmm_quaternion recentreToInitial;
	recentreToInitial = HMM_MultiplyQuaternion(HMM_QuaternionFromAxisAngle(UNIT_Y, yaw),
											   HMM_QuaternionFromAxisAngle(UNIT_X, pitch));

	// Create the inverse transform
	m_initialToRecentre = HMM_InverseQuaternion(recentreToInitial);
}

hmm_vec3 GyroTracker_GetForwards() {
	// This takes the forwards vector relative to the controller (negative Z
	// faces away from the player when it's held flat), transforms it to the
	// initial coordinate system, then transforms it to the recentre system.
	//
	// The result is the forwards vector, relative to the recentre frame.
	//
	// Here's a longer proof, for when we used to set initialToRecentre=inverse(currentToInitial),
	// in case you're interested:
	//
	// The multiplication order for quaternions with vectors matches matrices with vectors:
	// (q1 * q2) * v = q1 * (q2 * v)
	//
	// Consider the path the controller took as a series of rotations q1, q2, ..., qn:
	//
	// current to initial = q1 * q2 * ... * qn-1 * qn
	//
	// (Note the latest changes are at the right, and thus applied first: they're supposed
	//  to be relative to their parent, but all transforms are done relative to the world
	//  space the quaternion lives in. Thus we do them last when the world space matches
	//  their parent space. This is the same as how building any local->world matrix works.)
	//
	// If the controller was recentred at some m<n then q1, q2, ..., qm is a subset of the
	// full series that builds our rotation.
	//
	// recentre to initial= q1 * q2 * ... * qm-1 * qm
	//
	// The inversion rules for a product of quaternions is the same as for a product of matrices:
	//
	// initial to recentre = qm^-1 * qm-1^-1 * ... * q2^-1 * q1*-1
	//
	// Multiplying these two gives:
	//
	// initial to recentre * current to initial = (qm^-1 * qm-1^-1 * ... * q2^-1 * q1*-1) * (q1 * q2 * ... * qn-1 * qn)
	//
	// Cancelling the inverses:
	//
	// initial to recentre * current to initial = (qm * qm+1 * ... * qn-1 * qn)
	//
	// This is the transform we're looking for.
	// We then turn multiply by {0,0,-1} which is the axis coming out the USB port, transforming
	// this vector to apply all the rotations made since the last recentre operation.

	hmm_quaternion currentToRecentre = HMM_MultiplyQuaternion(m_initialToRecentre, m_currentToInitial);
	return HMM_MultiplyVec3f(GetMatrixColumn(HMM_QuaternionToMat4(currentToRecentre), 2), -1);
}
