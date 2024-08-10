//
// Created by Campbell on 23/07/2024.
//

#pragma once

#include "../refresh/gl3/header/HandmadeMath.h"

#ifdef __cplusplus

/**
 * This class integrates sensor inputs into a output direction vector.
 */
class GyroTracker {
public:
	void PushGyroEvent(uint64_t timestampNS, hmm_vec3 angularVelocity);

	void PushAccelerometerEvent(uint64_t timestampNS, hmm_vec3 acceleration);

	void Recentre();

	// The vector pointing out from where the controller's USB port normally is.
	// This maps to where the player should aim in-game.
	// This is relative to the recentre frame.
	// (this is only initialised in case it's somehow used before the first
	//  gyroscope reading is made, which could cause a divide-by-zero or NaN bug)
	hmm_vec3 GetForwards();

private:
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
	hmm_quaternion m_currentToInitial = {0, 0, 0, 1};

	// The rotation that transforms an object from the initial frame to the recentre frame.
	// This is stored inverted (vs being initial-to-recentred) since we'll use it every
	// time we read the forwards vector.
	hmm_quaternion m_initialToRecentre = {0, 0, 0, 1};

	hmm_vec3 m_lastAngVel = {0, 0, 0};
	uint64_t m_lastGyroTimestamp = 0;

	hmm_vec3 m_lastAccel = {0, 0, 0};
	uint64_t m_lastAccelerometerTimestamp = 0;

	// Buffer to detect when the controller is stationary, and find the gyro's DC bias
	// If we get many subsequent samples that are very very close, we'll assume the
	// controller is stationary.
	float m_biasDetectionBuffer[500] = {0};
	int m_biasDetectionIndex = 0;
	hmm_vec3 m_biasDetectionRunningSum = {0, 0, 0};
	hmm_vec3 m_gyroBias = {0, 0, 0};
};

#endif

// C wrapper functions
#ifdef __cplusplus
extern "C" {
#endif

void GyroTracker_PushGyroEvent(uint64_t timestampNS, hmm_vec3 angularVelocity);
void GyroTracker_PushAccelerometerEvent(uint64_t timestampNS, hmm_vec3 acceleration);
void GyroTracker_Recentre();
hmm_vec3 GyroTracker_GetForwards();

#ifdef __cplusplus
};
#endif
