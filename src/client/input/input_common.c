//
// Created by Campbell on 13/08/2024.
//

#include "input_common.h"

#include "header/input.h"
#include "../header/client.h"

// ----

// Maximal mouse move per frame
#define MOUSE_MAX 3000

// Minimal mouse move per frame
#define MOUSE_MIN 40

// ----

static qboolean mlooking;

// Console Variables
cvar_t *freelook;
cvar_t *lookstrafe;
cvar_t *m_forward;
cvar_t *m_pitch;
cvar_t *m_side;
cvar_t *m_up;
cvar_t *m_yaw;
static cvar_t *sensitivity;

static cvar_t *exponential_speedup;
static cvar_t *m_filter;

// Joystick sensitivity
static cvar_t *joy_yawsensitivity;
static cvar_t *joy_pitchsensitivity;
static cvar_t *joy_forwardsensitivity;
static cvar_t *joy_sidesensitivity;

// Joystick's analog sticks configuration
cvar_t *joy_layout;
static cvar_t *joy_left_expo;
static cvar_t *joy_left_snapaxis;
static cvar_t *joy_left_deadzone;
static cvar_t *joy_right_expo;
static cvar_t *joy_right_snapaxis;
static cvar_t *joy_right_deadzone;
static cvar_t *joy_flick_threshold;
static cvar_t *joy_flick_smoothed;

// Gyro mode (0=off, 3=on, 1-2=uses button to enable/disable)
cvar_t *gyro_mode;
cvar_t *gyro_turning_axis;    // yaw or roll

// Gyro sensitivity
static cvar_t *gyro_yawsensitivity;
static cvar_t *gyro_pitchsensitivity;
static cvar_t *gyro_tightening;

// Gyro is being used in this very moment
static qboolean gyro_active = false;

// Flick Stick
#define FLICK_TIME 6        // number of frames it takes for a flick to execute
static float target_angle;    // angle to end up facing at the end of a flick
static unsigned short int flick_progress = FLICK_TIME;

// Flick Stick's rotation input samples to smooth out
#define MAX_SMOOTH_SAMPLES 8
static float flick_samples[MAX_SMOOTH_SAMPLES];
static unsigned short int front_sample = 0;


/*
 * Joystick vector magnitude
 */
static float
IN_StickMagnitude(thumbstick_t stick)
{
	return sqrtf((stick.x * stick.x) + (stick.y * stick.y));
}

/*
 * Scales "v" from [deadzone, 1] range to [0, 1] range, then inherits sign
 */
static float
IN_MapRange(float v, float deadzone, float sign)
{
	return ((v - deadzone) / (1 - deadzone)) * sign;
}

/*
 * Radial deadzone based on github.com/jeremiah-sypult/Quakespasm-Rift
 */
static thumbstick_t
IN_RadialDeadzone(thumbstick_t stick, float deadzone)
{
	thumbstick_t result = {0};
	float magnitude = Q_min(IN_StickMagnitude(stick), 1.0f);
	deadzone = Q_min(Q_max(deadzone, 0.0f), 0.9f);        // clamp to [0.0, 0.9]

	if (magnitude > deadzone)
	{
		const float scale = ((magnitude - deadzone) / (1.0 - deadzone)) / magnitude;
		result.x = stick.x * scale;
		result.y = stick.y * scale;
	}

	return result;
}

/*
 * Sloped axial deadzone based on github.com/Minimuino/thumbstick-deadzones
 * Provides a "snap-to-axis" feeling, without losing precision near the center of the stick
 */
static thumbstick_t
IN_SlopedAxialDeadzone(thumbstick_t stick, float deadzone)
{
	thumbstick_t result = {0};
	float abs_x = fabsf(stick.x);
	float abs_y = fabsf(stick.y);
	float sign_x = copysignf(1.0f, stick.x);
	float sign_y = copysignf(1.0f, stick.y);
	deadzone = Q_min(deadzone, 0.5f);
	float deadzone_x = deadzone * abs_y;    // deadzone of one axis depends...
	float deadzone_y = deadzone * abs_x;    // ...on the value of the other axis

	if (abs_x > deadzone_x)
	{
		result.x = IN_MapRange(abs_x, deadzone_x, sign_x);
	}
	if (abs_y > deadzone_y)
	{
		result.y = IN_MapRange(abs_y, deadzone_y, sign_y);
	}

	return result;
}

/*
 * Exponent applied on stick magnitude
 */
static thumbstick_t
IN_ApplyExpo(thumbstick_t stick, float exponent)
{
	thumbstick_t result = {0};
	float magnitude = IN_StickMagnitude(stick);
	if (magnitude == 0)
	{
		return result;
	}

	const float eased = powf(magnitude, exponent) / magnitude;
	result.x = stick.x * eased;
	result.y = stick.y * eased;
	return result;
}

/*
 * Minimize gyro movement when under a small threshold.
 * http://gyrowiki.jibbsmart.com/blog:good-gyro-controls-part-1:the-gyro-is-a-mouse#toc9
 */
static thumbstick_t
IN_TightenInput(float yaw, float pitch)
{
	thumbstick_t input = {yaw, pitch};
	const float magnitude = IN_StickMagnitude(input);
	const float threshold = (M_PI / 180.0f) * gyro_tightening->value;

	if (magnitude < threshold)
	{
		const float scale = magnitude / threshold;
		input.x *= scale;
		input.y *= scale;
	}
	return input;
}

/*
 * Delete flick stick's buffer of angle samples for smoothing
 */
static void
IN_ResetSmoothSamples()
{
	front_sample = 0;
	for (int i = 0; i < MAX_SMOOTH_SAMPLES; i++)
	{
		flick_samples[i] = 0.0f;
	}
}

/*
 * Soft tiered smoothing for angle rotations with Flick Stick
 * http://gyrowiki.jibbsmart.com/blog:tight-and-smooth:soft-tiered-smoothing
 */
static float
IN_SmoothedStickRotation(float value)
{
	float top_threshold = joy_flick_smoothed->value;
	float bottom_threshold = top_threshold / 2.0f;
	if (top_threshold == 0)
	{
		return value;
	}

	// sample in the circular smoothing buffer we want to write over
	front_sample = (front_sample + 1) % MAX_SMOOTH_SAMPLES;

	// if input > top threshold, it'll all be consumed immediately
	//				0 gets put into the smoothing buffer
	// if input < bottom threshold, it'll all be put in the smoothing buffer
	//				0 for immediate consumption
	float immediate_weight = (fabsf(value) - bottom_threshold)
							 / (top_threshold - bottom_threshold);
	immediate_weight = Q_min(Q_max(immediate_weight, 0.0f), 1.0f); // clamp to [0, 1] range

	// now we can push the smooth sample
	float smooth_weight = 1.0f - immediate_weight;
	flick_samples[front_sample] = value * smooth_weight;

	// calculate smoothed result
	float average = 0;
	for (int i = 0; i < MAX_SMOOTH_SAMPLES; i++)
	{
		average += flick_samples[i];
	}
	average /= MAX_SMOOTH_SAMPLES;

	// finally, add immediate portion (original input)
	return average + value * immediate_weight;
}

/*
 * Flick Stick handling: detect if the player just started one, or return the
 * player rotation if stick was already flicked
 */
static float
IN_FlickStick(thumbstick_t stick, float axial_deadzone)
{
	static qboolean is_flicking;
	static float last_stick_angle;
	thumbstick_t processed = stick;
	float angle_change = 0;

	if (IN_StickMagnitude(stick) > Q_min(joy_flick_threshold->value, 1.0f))    // flick!
	{
		// Make snap-to-axis only if player wasn't already flicking
		if (!is_flicking || flick_progress < FLICK_TIME)
		{
			processed = IN_SlopedAxialDeadzone(stick, axial_deadzone);
		}

		const float stick_angle = (180 / M_PI) * atan2f(-processed.x, -processed.y);

		if (!is_flicking)
		{
			// Flicking begins now, with a new target
			is_flicking = true;
			flick_progress = 0;
			target_angle = stick_angle;
			IN_ResetSmoothSamples();
		}
		else
		{
			// Was already flicking, just turning now
			angle_change = stick_angle - last_stick_angle;

			// angle wrap: https://stackoverflow.com/a/11498248/1130520
			angle_change = fmod(angle_change + 180.0f, 360.0f);
			if (angle_change < 0)
			{
				angle_change += 360.0f;
			}
			angle_change -= 180.0f;
			angle_change = IN_SmoothedStickRotation(angle_change);
		}

		last_stick_angle = stick_angle;
	}
	else
	{
		is_flicking = false;
	}

	return angle_change;
}

/*
 * Look down
 */
static void
IN_MLookDown(void)
{
	mlooking = true;
}

/*
 * Look up
 */
static void
IN_MLookUp(void)
{
	mlooking = false;
	IN_CenterView();
}

static void
IN_JoyAltSelectorDown(void)
{
	joy_altselector_pressed = true;
}

static void
IN_JoyAltSelectorUp(void)
{
	joy_altselector_pressed = false;
}

static void
IN_GyroActionDown(void)
{
	switch ((int) gyro_mode->value)
	{
		case 1:
			gyro_active = true;
			return;
		case 2:
			gyro_active = false;
	}
}

static void
IN_GyroActionUp(void)
{
	switch ((int) gyro_mode->value)
	{
		case 1:
			gyro_active = false;
			return;
		case 2:
			gyro_active = true;
	}
}

void IN_Common_Init(void)
{
	exponential_speedup = Cvar_Get("exponential_speedup", "0", CVAR_ARCHIVE);
	freelook = Cvar_Get("freelook", "1", CVAR_ARCHIVE);
	lookstrafe = Cvar_Get("lookstrafe", "0", CVAR_ARCHIVE);
	m_filter = Cvar_Get("m_filter", "0", CVAR_ARCHIVE);
	m_up = Cvar_Get("m_up", "1", CVAR_ARCHIVE);
	m_forward = Cvar_Get("m_forward", "1", CVAR_ARCHIVE);
	m_pitch = Cvar_Get("m_pitch", "0.022", CVAR_ARCHIVE);
	m_side = Cvar_Get("m_side", "0.8", CVAR_ARCHIVE);
	m_yaw = Cvar_Get("m_yaw", "0.022", CVAR_ARCHIVE);
	sensitivity = Cvar_Get("sensitivity", "3", CVAR_ARCHIVE);

	joy_yawsensitivity = Cvar_Get("joy_yawsensitivity", "1.0", CVAR_ARCHIVE);
	joy_pitchsensitivity = Cvar_Get("joy_pitchsensitivity", "1.0", CVAR_ARCHIVE);
	joy_forwardsensitivity = Cvar_Get("joy_forwardsensitivity", "1.0", CVAR_ARCHIVE);
	joy_sidesensitivity = Cvar_Get("joy_sidesensitivity", "1.0", CVAR_ARCHIVE);

	joy_layout = Cvar_Get("joy_layout", "0", CVAR_ARCHIVE);
	joy_left_expo = Cvar_Get("joy_left_expo", "2.0", CVAR_ARCHIVE);
	joy_left_snapaxis = Cvar_Get("joy_left_snapaxis", "0.15", CVAR_ARCHIVE);
	joy_left_deadzone = Cvar_Get("joy_left_deadzone", "0.16", CVAR_ARCHIVE);
	joy_right_expo = Cvar_Get("joy_right_expo", "2.0", CVAR_ARCHIVE);
	joy_right_snapaxis = Cvar_Get("joy_right_snapaxis", "0.15", CVAR_ARCHIVE);
	joy_right_deadzone = Cvar_Get("joy_right_deadzone", "0.16", CVAR_ARCHIVE);
	joy_flick_threshold = Cvar_Get("joy_flick_threshold", "0.65", CVAR_ARCHIVE);
	joy_flick_smoothed = Cvar_Get("joy_flick_smoothed", "8.0", CVAR_ARCHIVE);

	gyro_yawsensitivity = Cvar_Get("gyro_yawsensitivity", "1.0", CVAR_ARCHIVE);
	gyro_pitchsensitivity = Cvar_Get("gyro_pitchsensitivity", "1.0", CVAR_ARCHIVE);
	gyro_tightening = Cvar_Get("gyro_tightening", "3.5", CVAR_ARCHIVE);
	gyro_turning_axis = Cvar_Get("gyro_turning_axis", "0", CVAR_ARCHIVE);

	gyro_mode = Cvar_Get("gyro_mode", "2", CVAR_ARCHIVE);
	if ((int) gyro_mode->value == 2)
	{
		gyro_active = true;
	}

	Cmd_AddCommand("+mlook", IN_MLookDown);
	Cmd_AddCommand("-mlook", IN_MLookUp);

	Cmd_AddCommand("+joyaltselector", IN_JoyAltSelectorDown);
	Cmd_AddCommand("-joyaltselector", IN_JoyAltSelectorUp);
	Cmd_AddCommand("+gyroaction", IN_GyroActionDown);
	Cmd_AddCommand("-gyroaction", IN_GyroActionUp);
}

void IN_Common_Move(usercmd_t *cmd, float mouse_x, float mouse_y, thumbstick_t left_stick, thumbstick_t right_stick,
					float gyro_x, float gyro_y, float gyro_z)
{
	// Flick Stick's factors to change to the target angle with a feeling of "ease out"
	static const float rotation_factor[FLICK_TIME] = {
			0.305555556f, 0.249999999f, 0.194444445f, 0.138888889f, 0.083333333f, 0.027777778f
	};

	static float old_mouse_x;
	static float old_mouse_y;
	static float joystick_yaw, joystick_pitch;
	static float joystick_forwardmove, joystick_sidemove;
	thumbstick_t gyro_in = {0};

	if (m_filter->value)
	{
		if ((mouse_x > 1) || (mouse_x < -1))
		{
			mouse_x = (mouse_x + old_mouse_x) * 0.5;
		}

		if ((mouse_y > 1) || (mouse_y < -1))
		{
			mouse_y = (mouse_y + old_mouse_y) * 0.5;
		}
	}

	old_mouse_x = mouse_x;
	old_mouse_y = mouse_y;

	if (mouse_x || mouse_y)
	{
		if (!exponential_speedup->value)
		{
			mouse_x *= sensitivity->value;
			mouse_y *= sensitivity->value;
		}
		else
		{
			if ((mouse_x > MOUSE_MIN) || (mouse_y > MOUSE_MIN) ||
				(mouse_x < -MOUSE_MIN) || (mouse_y < -MOUSE_MIN))
			{
				mouse_x = (mouse_x * mouse_x * mouse_x) / 4;
				mouse_y = (mouse_y * mouse_y * mouse_y) / 4;

				if (mouse_x > MOUSE_MAX)
				{
					mouse_x = MOUSE_MAX;
				}
				else if (mouse_x < -MOUSE_MAX)
				{
					mouse_x = -MOUSE_MAX;
				}

				if (mouse_y > MOUSE_MAX)
				{
					mouse_y = MOUSE_MAX;
				}
				else if (mouse_y < -MOUSE_MAX)
				{
					mouse_y = -MOUSE_MAX;
				}
			}
		}

		// add mouse X/Y movement to cmd
		if ((in_strafe.state & 1) || (lookstrafe->value && mlooking))
		{
			cmd->sidemove += m_side->value * mouse_x;
		}
		else
		{
			cl.viewangles[YAW] -= m_yaw->value * mouse_x;
		}

		if ((mlooking || freelook->value) && !(in_strafe.state & 1))
		{
			cl.viewangles[PITCH] += m_pitch->value * mouse_y;
		}
		else
		{
			cmd->forwardmove -= m_forward->value * mouse_y;
		}
	}

	if (left_stick.x || left_stick.y)
	{
		left_stick = IN_RadialDeadzone(left_stick, joy_left_deadzone->value);
		if ((int) joy_layout->value == LAYOUT_FLICK_STICK_SOUTHPAW)
		{
			cl.viewangles[YAW] += IN_FlickStick(left_stick, joy_left_snapaxis->value);
		}
		else
		{
			left_stick = IN_SlopedAxialDeadzone(left_stick, joy_left_snapaxis->value);
			left_stick = IN_ApplyExpo(left_stick, joy_left_expo->value);
		}
	}

	if (right_stick.x || right_stick.y)
	{
		right_stick = IN_RadialDeadzone(right_stick, joy_right_deadzone->value);
		if ((int) joy_layout->value == LAYOUT_FLICK_STICK)
		{
			cl.viewangles[YAW] += IN_FlickStick(right_stick, joy_right_snapaxis->value);
		}
		else
		{
			right_stick = IN_SlopedAxialDeadzone(right_stick, joy_right_snapaxis->value);
			right_stick = IN_ApplyExpo(right_stick, joy_right_expo->value);
		}
	}

	switch ((int) joy_layout->value)
	{
		case LAYOUT_SOUTHPAW:
			joystick_forwardmove = right_stick.y;
			joystick_sidemove = right_stick.x;
			joystick_yaw = left_stick.x;
			joystick_pitch = left_stick.y;
			break;
		case LAYOUT_LEGACY:
			joystick_forwardmove = left_stick.y;
			joystick_sidemove = right_stick.x;
			joystick_yaw = left_stick.x;
			joystick_pitch = right_stick.y;
			break;
		case LAYOUT_LEGACY_SOUTHPAW:
			joystick_forwardmove = right_stick.y;
			joystick_sidemove = left_stick.x;
			joystick_yaw = right_stick.x;
			joystick_pitch = left_stick.y;
			break;
		case LAYOUT_FLICK_STICK:    // yaw already set by now
			joystick_forwardmove = left_stick.y;
			joystick_sidemove = left_stick.x;
			break;
		case LAYOUT_FLICK_STICK_SOUTHPAW:
			joystick_forwardmove = right_stick.y;
			joystick_sidemove = right_stick.x;
			break;
		default:    // LAYOUT_DEFAULT
			joystick_forwardmove = left_stick.y;
			joystick_sidemove = left_stick.x;
			joystick_yaw = right_stick.x;
			joystick_pitch = right_stick.y;
	}

	// To make the the viewangles changes independent of framerate we need to scale
	// with frametime (assuming the configured values are for 60hz)
	//
	// For movement this is not needed, as those are absolute values independent of framerate
	float joyViewFactor = cls.rframetime / 0.01666f;
	float gyroViewFactor = (1.0f / M_PI) * joyViewFactor;

	if (joystick_yaw)
	{
		cl.viewangles[YAW] -= (m_yaw->value * joy_yawsensitivity->value
							   * cl_yawspeed->value * joystick_yaw) * joyViewFactor;
	}

	if (joystick_pitch)
	{
		cl.viewangles[PITCH] += (m_pitch->value * joy_pitchsensitivity->value
								 * cl_pitchspeed->value * joystick_pitch) * joyViewFactor;
	}

	if (joystick_forwardmove)
	{
		// We need to be twice as fast because with joystick we run...
		cmd->forwardmove -= m_forward->value * joy_forwardsensitivity->value
							* cl_forwardspeed->value * 2.0f * joystick_forwardmove;
	}

	if (joystick_sidemove)
	{
		// We need to be twice as fast because with joystick we run...
		cmd->sidemove += m_side->value * joy_sidesensitivity->value
						 * cl_sidespeed->value * 2.0f * joystick_sidemove;
	}

	float gyro_pitch = 0;
	float gyro_yaw = 0;

	if (gyro_active && gyro_mode->value && !cl_paused->value && cls.key_dest == key_game)
	{
		if (!gyro_turning_axis->value)
		{
			gyro_yaw = gyro_y;        // yaw
		}
		else
		{
			gyro_yaw = -gyro_z;    // roll
		}
		gyro_pitch = gyro_x;
	}

	if (gyro_yaw || gyro_pitch)
	{
		gyro_in = IN_TightenInput(gyro_yaw, gyro_pitch);
	}

	if (gyro_in.x)
	{
		cl.viewangles[YAW] += m_yaw->value * gyro_yawsensitivity->value
							  * cl_yawspeed->value * gyro_in.x * gyroViewFactor;
	}

	if (gyro_in.y)
	{
		cl.viewangles[PITCH] -= m_pitch->value * gyro_pitchsensitivity->value
								* cl_pitchspeed->value * gyro_in.y * gyroViewFactor;
	}

	// Flick Stick: flick in progress, changing the yaw angle to the target progressively
	if (flick_progress < FLICK_TIME)
	{
		cl.viewangles[YAW] += target_angle * rotation_factor[flick_progress];
		flick_progress++;
	}
}
