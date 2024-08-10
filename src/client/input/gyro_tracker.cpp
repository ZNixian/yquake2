//
// Created by Campbell on 23/07/2024.
//

#define HANDMADE_MATH_IMPLEMENTATION

#include "gyro_tracker.h"

#include <stdint.h> // NOLINT(*-deprecated-headers)
#include <array>

void GyroTracker::PushGyroEvent(uint64_t timestampNS, hmm_vec3 angularVelocity) {
	// If there's a big time gap, assume that something interrupted
	// the data and we can't meaningfully integrate over that gap.
	uint64_t deltaNS = timestampNS - m_lastGyroTimestamp;
	m_lastGyroTimestamp = timestampNS;
	if (deltaNS > 500'000'000) {
		m_lastAngVel = angularVelocity;
		return;
	}

	// Assume the true angular velocity was linearly interpolated between
	// the previous and current values.
	float deltaS = (float) deltaNS / 1'000'000'000.f;

	// This is what the integral of f=a(1-t/T)+b(t/T) works out to, where t
	// is the time since the previous sample and T is the period between
	// that sample (a) and the current sample (b).
	hmm_vec3 averageAngVel = (m_lastAngVel + angularVelocity) / 2.f - m_gyroBias;
	hmm_vec3 integratedEuler = averageAngVel * deltaS;

	// Convert the Euler rotation angles to a quaternion
	// TODO is the rotation order here an accuracy problem?
	hmm_quaternion thisUpdateRotation = HMM_QuaternionFromAxisAngle(hmm_vec3{1, 0, 0}, integratedEuler.X);
	thisUpdateRotation = thisUpdateRotation * HMM_QuaternionFromAxisAngle(hmm_vec3{0, 1, 0}, integratedEuler.Y);
	thisUpdateRotation = thisUpdateRotation * HMM_QuaternionFromAxisAngle(hmm_vec3{0, 0, 1}, integratedEuler.Z);

	m_currentToInitial = m_currentToInitial * thisUpdateRotation;

	m_lastAngVel = angularVelocity;

	// Disabled: quake already does this
	/*
	// Track the angular velocities, if they remain constant for awhile then assume the controller
	// is stationary and use that to find the gyro's DC bias.
	// We assume that two angular velocities are similar if they have a similar norm,
	// so we don't detect if the direction is slowly changing. As long as we're strict
	// enough with our tolerance though, this shouldn't be a problem.
	m_biasDetectionBuffer[m_biasDetectionIndex++] = HMM_Length(angularVelocity);
	m_biasDetectionRunningSum += angularVelocity;
	if (m_biasDetectionIndex >= std::size(m_biasDetectionBuffer)) {
		// By keeping a sum of all the values we've put into the buffer, we get
		// an index we can use without having to scan the buffer a second time.
		// This also lets us store the norms of the vectors in the buffer, rather than
		// the vectors themselves.
		hmm_vec3 average = m_biasDetectionRunningSum / (float) m_biasDetectionIndex;
		float averageNorm = HMM_Length(average);

		float tolerance = 0.01;
		float minReq = averageNorm * (1 - tolerance);
		float maxReq = averageNorm * (1 + tolerance);
		int outsideToleranceCount = 0;

		int blockCount = 10;
		int blockSize = (int) std::size(m_biasDetectionBuffer) / blockCount;
		for (int i = 0; i < 10; i++) {
			float sum = 0;
			for (int j = 0; j < blockSize; ++j) {
				sum += m_biasDetectionBuffer[i * blockSize + j];
			}

			float norm = sum / (float) blockSize;
			if (norm < minReq || norm > maxReq) {
				outsideToleranceCount++;
			}
		}

		// printf("%d\n", outsideToleranceCount);
		if (outsideToleranceCount == 0) {
			m_gyroBias = average;
		}

		m_biasDetectionIndex = 0;
		m_biasDetectionRunningSum = {0, 0, 0};
	}
	*/
}

void GyroTracker::PushAccelerometerEvent(uint64_t timestampNS, hmm_vec3 acceleration) {
	// If there's a big time gap, assume that something interrupted
	// the data and we can't meaningfully integrate over that gap.
	uint64_t deltaNS = timestampNS - m_lastAccelerometerTimestamp;
	m_lastAccelerometerTimestamp = timestampNS;
	if (deltaNS > 500'000'000) {
		m_lastAccel = acceleration;
		return;
	}

	float deltaS = (float) deltaNS / 1'000'000'000.f;

	float accelerationMagnitude = HMM_Length(acceleration);
	if (accelerationMagnitude == 0) {
		return; // Block divide-by-zeros
	}

	// Find the normalised vector representing where the acceleration is, which we assume
	// is where gravity is - accelerations from the player shaking the controller should
	// approximately cancel out.
	// TODO find a filtered controller-relative gravity vector, and use that for rotation.
	hmm_vec3 gravity = acceleration / accelerationMagnitude;

	// Find out where we gravity *should* be, from our definition of the initial reference
	// frame being horizontally aligned.
	// We use the inverse to convert from the initial frame to the current frame.
	hmm_quaternion initialToCurrent = HMM_InverseQuaternion(m_currentToInitial);
	hmm_vec3 down = (HMM_QuaternionToMat4(initialToCurrent) * hmm_vec4{0, -1, 0, 0}).XYZ;

	// Find the angle-axis difference between these two vectors.
	hmm_vec3 cross = HMM_Cross(down, gravity);
	float length = HMM_Length(cross);
	if (length == 0) {
		return; // No drift at all! Block the divide-by-zero.
	}
	float angle = asinf(length);
	hmm_vec3 axis = cross / length;

	// Figure out how much to rotate the state by to correct for the gyro drift
	// This effectively forms an IIR filter.
	float angleToCorrect = angle * deltaS;
	hmm_quaternion correction = HMM_QuaternionFromAxisAngle(axis, angleToCorrect);
	m_currentToInitial = m_currentToInitial * correction;
}

void GyroTracker::Recentre() {
	// Find the player's yaw, relative to the initial frame
	// +ve is CCW when viewed from above
	hmm_vec3 forwards = (HMM_QuaternionToMat4(m_currentToInitial) * hmm_vec4{0, 0, -1, 0}).XYZ;
	float yaw = atan2f(-forwards.X, -forwards.Z);

	// Find the player's pitch, where +ve is above the horizon and -ve is below
	float pitch = asinf(forwards.Y);

	// Find the local->world transform just based on these, ignoring the roll.
	// Note we *have* to include the yaw here if we want to cancel out the pitch - because
	// the plane through which the pitch acts varies depending on the yaw, if we don't
	// include the yaw then weird things happen, which is most noticeable with recentring
	// not putting the camera back to the horizon.
	hmm_quaternion recentreToInitial;
	recentreToInitial =
			HMM_QuaternionFromAxisAngle(hmm_vec3{0, 1, 0}, yaw) * HMM_QuaternionFromAxisAngle(hmm_vec3{1, 0, 0}, pitch);

	// Create the inverse transform
	m_initialToRecentre = HMM_InverseQuaternion(recentreToInitial);
}

hmm_vec3 GyroTracker::GetForwards() {
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

	return (HMM_QuaternionToMat4(m_initialToRecentre * m_currentToInitial) * hmm_vec4{0, 0, -1, 0}).XYZ;
}

// C wrappers

static GyroTracker impl;

void GyroTracker_PushGyroEvent(uint64_t timestampNS, hmm_vec3 angularVelocity) {
	impl.PushGyroEvent(timestampNS, angularVelocity);
}

void GyroTracker_PushAccelerometerEvent(uint64_t timestampNS, hmm_vec3 acceleration) {
	impl.PushAccelerometerEvent(timestampNS, acceleration);
}

void GyroTracker_Recentre() {
	impl.Recentre();
}

hmm_vec3 GyroTracker_GetForwards() {
	return impl.GetForwards();
}
