//
// Created by Campbell on 13/08/2024.
//

#pragma once

#include "../../common/header/shared.h"

enum
{
	LAYOUT_DEFAULT = 0,
	LAYOUT_SOUTHPAW,
	LAYOUT_LEGACY,
	LAYOUT_LEGACY_SOUTHPAW,
	LAYOUT_FLICK_STICK,
	LAYOUT_FLICK_STICK_SOUTHPAW
};

typedef struct
{
	float x;
	float y;
} thumbstick_t;

void IN_Common_Init(void);

void IN_Common_Move(usercmd_t *cmd, float mouse_x, float mouse_y, thumbstick_t left_stick, thumbstick_t right_stick,
					float gyro_x, float gyro_y, float gyro_z);
