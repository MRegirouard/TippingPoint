/**
 * Project ports and settings. This file should be used so that
 * changing settings is easy, and to avoid arbitrary constants
 * throught the code.
 */

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

// Controls
#define DRIVE_FORWARD ANALOG_RIGHT_Y
#define DRIVE_TURN ANALOG_RIGHT_X
#define DRIVE_SLOW_FORWARD ANALOG_LEFT_Y
#define DRIVE_SLOW_TURN ANALOG_LEFT_X

#define REVERSE_DRIVE_CONTROL DIGITAL_RIGHT

#define FRONT_ARM_UP_CONTROL DIGITAL_R1
#define FRONT_ARM_DOWN_CONTROL DIGITAL_R2

#define BACK_ARM_UP_CONTROL DIGITAL_L1
#define BACK_ARM_DOWN_CONTROL DIGITAL_L2

#define CLAW_OPEN_CONTROL DIGITAL_UP
#define CLAW_CLOSE_CONTROL DIGITAL_DOWN

#define AUTO_BALANCE_CONTROL DIGITAL_LEFT

#define SELECTION_SCREEN_UP DIGITAL_X
#define SELECTION_SCREEN_DOWN DIGITAL_B
#define SELECTION_SCREEN_CONFIRM DIGITAL_A

#define CONTROLLER2_FRONT_ARM ANALOG_RIGHT_Y
#define CONTROLLER2_BACK_ARM ANALOG_LEFT_Y

#define BRAKE_TOGGLE_CONTROL DIGITAL_A

// Drive
#define FR_PORT 18
#define FR_REVERSED false
#define FL_PORT 11
#define FL_REVERSED true
#define BR_PORT 20
#define BR_REVERSED false
#define BL_PORT 12
#define BL_REVERSED true

#define WHEEL_DIAMETER 4.25
#define WHEELBASE 16
#define DRIVE_GEAR_RATIO 7.0 / 5.0
#define DRIVE_ACCEL_LIMIT 400
#define DRIVE_DEADZONE 10
#define DRIVE_TURN_SCALE 1.0 / 1.5
#define DRIVE_SLOW_SCALE 1.0 / 3.0
#define DRIVE_SLOW_TURN_SCALE 1.0 / 3.0
#define DRIVE_START_REVERSE_STATE false

// Front Arm
#define FRONT_ARM_LEFT_PORT 4
#define FRONT_ARM_LEFT_REVERSED true
#define FRONT_ARM_RIGHT_PORT 7
#define FRONT_ARM_RIGHT_REVERSED false

#define FRONT_ARM_GEAR_RATIO 12.0 / 60.0
#define FRONT_ARM_ACCEL_LIMIT 300
#define FRONT_ARM_UP_LIMIT_PORT 1
#define FRONT_ARM_RAISE_SPEED 100
#define FRONT_ARM_LOWER_SPEED 100

#define FRONT_ARM_INITIAL_POSITION 98
#define FRONT_ARM_HOLD_POSITION 60
#define FRONT_ARM_DOWN_POSITION 0
#define FRONT_ARM_DRIVE_TIMEOUT 700

// Back Arm
#define BACK_ARM_LEFT_PORT 9
#define BACK_ARM_LEFT_REVERSED true
#define BACK_ARM_RIGHT_PORT 3
#define BACK_ARM_RIGHT_REVERSED false

#define BACK_ARM_GEAR_RATIO 12.0 / 60.0
#define BACK_ARM_ACCEL_LIMIT 300
#define BACK_ARM_UP_LIMIT_PORT 3
#define BACK_ARM_RAISE_SPEED 100
#define BACK_ARM_LOWER_SPEED 100

#define BACK_ARM_INITIAL_POSITION 0
#define BACK_ARM_UP_POSITION 120
#define BACK_ARM_HOLD_POSITION 30

// Claw
#define CLAW_VALVE_PORT 2
#define CLAW_OPEN HIGH
#define CLAW_CLOSE LOW

// IMU
#define IMU_PORT 10

// Front Vision
#define FRONT_VISION_PORT 1
#define FRONT_VISION_SIGNATURE -1, 1915, 956, -4153, -3373, -3764, 2.5, 0
#define FRONT_VISION_SIGNATURE_INDEX 1
#define FRONT_RED_VISION_SIGNATURE 5115, 8381, 6748, -1219, -781, -1000, 3.000, 0
#define FRONT_RED_VISION_SIGNATURE_INDEX 2
#define FRONT_BLUE_VISION_SIGNATURE -2905, -1745, -2325, 6101, 10811, 8456, 3.000, 0
#define FRONT_BLUE_VISION_SIGNATURE_INDEX 3

// Back Vision
#define BACK_VISION_PORT 2
#define BACK_VISION_SIGNATURE FRONT_VISION_SIGNATURE
#define BACK_VISION_SIGNATURE_INDEX 1

// PID Tuning
#define FRONT_OBJ_TRACK_PID 0.7, 0, 2
#define BALANCE_PID 0.4, 0.5, 3

// MISC
#define LOOP_DELAY 20
#define PRINT_LOOP_DELAY 200

#endif // _SETTINGS_H_