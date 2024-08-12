//
// Created by Campbell on 23/07/2024.
//

#pragma once

#include "../refresh/gl3/header/HandmadeMath.h"

void GyroTracker_PushGyroEvent(uint64_t timestampNS, hmm_vec3 angularVelocity);

void GyroTracker_PushAccelerometerEvent(uint64_t timestampNS, hmm_vec3 acceleration);

void GyroTracker_Recentre();

// The vector pointing out from where the controller's USB port normally is.
// This maps to where the player should aim in-game.
// This is relative to the recentre frame.
// (this is only initialised in case it's somehow used before the first
//  gyroscope reading is made, which could cause a divide-by-zero or NaN bug)
hmm_vec3 GyroTracker_GetForwards();
