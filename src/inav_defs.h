#pragma once

#define BOARD_INFO_FIXED_SIZE (4 + 2 + 1 + 1 + 1) // Size before variable name
#define MAX_NAME_LENGTH 47 // Check firmware source for actual limit
#define NAV_MAX_WAYPOINTS       120
#define MAX_BOX_IDS 70 // Estimate, actual depends on INAV version/config
#define MAX_GVARS 16 // Example typical value, check firmware
#define MAX_LOGIC_CONDITIONS 64
#define MAX_MODE_RANGES 40 // ?

#define MAX_SUPPORTED_RC_CHANNEL_COUNT 34
#define MAX_RC_CHANNELS 18 // Common RX channel limit
#define MAX_SUPPORTED_CHANNELS 18 ////////////////

#define MAX_SUPPORTED_SERVOS 18
#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SUPPORTED_MOTORS 18
#define MAX_SERVO_RULES (2 * MAX_SUPPORTED_SERVOS)
#define MAX_SERVO_SPEED UINT8_MAX
#define SERVO_OUTPUT_MAX 2500
#define SERVO_OUTPUT_MIN 500

#define STICK_CHANNEL_COUNT 4

#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE (PWM_RANGE_MIN + ((PWM_RANGE_MAX - PWM_RANGE_MIN) / 2))

#define PWM_PULSE_MIN   750       // minimum PWM pulse width which is considered valid
#define PWM_PULSE_MAX   2250      // maximum PWM pulse width which is considered valid

#define MIDRC_MIN 1200
#define MIDRC_MAX 1700