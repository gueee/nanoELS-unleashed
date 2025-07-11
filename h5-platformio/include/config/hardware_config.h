/*
 * hardware_config.h
 * Hardware Configuration for nanoELS H5
 * 
 * Contains all pin definitions, constants, and hardware-specific configuration
 * for the ESP32-S3 based lathe controller.
 */

#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>

// Hardware version and software version
#define HARDWARE_VERSION 5
#define SOFTWARE_VERSION 9

// WiFi configuration
#define WIFI_ENABLED true
#define SSID "your-wifi-name"
#define PASSWORD "your-password"

// Encoder configuration
#define ENCODER_PPR 1200                    // 1200 step spindle optical rotary encoder
#define ENCODER_BACKLASH 3                  // Number of impulses encoder can issue without movement
#define ENCODER_FILTER 1                    // Encoder pulses shorter than this will be ignored
#define PCNT_LIM 31000                      // Limit used in hardware pulse counter logic
#define PCNT_CLEAR 30000                    // Limit where we reset hardware pulse counter value

// Encoder pins
#define ENC_A 13                            // Spindle encoder A pin
#define ENC_B 14                            // Spindle encoder B pin

// Z axis configuration (main lead screw)
#define SCREW_Z_DU 40000                    // 4mm SFU1204 ball screw in deci-microns
#define MOTOR_STEPS_Z 800                   // Motor steps per revolution
#define SPEED_START_Z MOTOR_STEPS_Z         // Initial speed (steps/sec)
#define ACCELERATION_Z 25 * MOTOR_STEPS_Z   // Acceleration (steps/sec²)
#define SPEED_MANUAL_MOVE_Z 8 * MOTOR_STEPS_Z // Maximum manual move speed
#define INVERT_Z false                      // Invert direction
#define INVERT_Z_ENABLE false               // Invert enable pin
#define NEEDS_REST_Z false                  // Set to false for closed-loop drivers
#define MAX_TRAVEL_MM_Z 300                 // Maximum travel distance (mm)
#define BACKLASH_DU_Z 0                     // Backlash in deci-microns
#define NAME_Z 'Z'                          // Axis name

// Z axis pins
#define Z_ENA 41                            // Z axis enable pin
#define Z_DIR 42                            // Z axis direction pin
#define Z_STEP 35                           // Z axis step pin
#define Z_PULSE_A 18                        // Z axis MPG A pin
#define Z_PULSE_B 8                         // Z axis MPG B pin

// X axis configuration (cross-slide)
#define SCREW_X_DU 40000                    // 4mm SFU1204 ball screw in deci-microns
#define MOTOR_STEPS_X 800                   // Motor steps per revolution
#define SPEED_START_X MOTOR_STEPS_X         // Initial speed (steps/sec)
#define ACCELERATION_X 25 * MOTOR_STEPS_X   // Acceleration (steps/sec²)
#define SPEED_MANUAL_MOVE_X 8 * MOTOR_STEPS_X // Maximum manual move speed
#define INVERT_X true                       // Invert direction
#define INVERT_X_ENABLE false               // Invert enable pin
#define NEEDS_REST_X false                  // Set to false for all kinds of drivers
#define MAX_TRAVEL_MM_X 100                 // Maximum travel distance (mm)
#define BACKLASH_DU_X 0                     // Backlash in deci-microns
#define NAME_X 'X'                          // Axis name

// X axis pins
#define X_ENA 16                            // X axis enable pin
#define X_DIR 15                            // X axis direction pin
#define X_STEP 7                            // X axis step pin
#define X_PULSE_A 47                        // X axis MPG A pin
#define X_PULSE_B 21                        // X axis MPG B pin

// Y axis configuration (optional)
#define ACTIVE_Y false                       // Whether Y axis is connected
#define ROTARY_Y true                       // Whether Y axis is rotary
#define SCREW_Y_DU 20000                    // Degrees multiplied by 10000 per worm gear turn
#define MOTOR_STEPS_Y 300                   // Motor steps per revolution
#define SPEED_START_Y 1600                  // Initial speed (steps/sec)
#define ACCELERATION_Y 16000                // Acceleration (steps/sec²)
#define SPEED_MANUAL_MOVE_Y 3200            // Maximum manual move speed
#define INVERT_Y false                      // Invert direction
#define INVERT_Y_ENABLE false               // Invert enable pin
#define NEEDS_REST_Y false                  // Set to false for closed-loop drivers
#define MAX_TRAVEL_MM_Y 360                 // Maximum travel distance (degrees)
#define BACKLASH_DU_Y 0                     // Backlash in deci-microns
#define NAME_Y 'Y'                          // Axis name

// Y axis pins
#define Y_ENA 1                             // Y axis enable pin
#define Y_DIR 2                             // Y axis direction pin
#define Y_STEP 17                           // Y axis step pin
#define Y_PULSE_A 45                        // Y axis MPG A pin
#define Y_PULSE_B 48                        // Y axis MPG B pin

// PS2 keyboard pins
#define KEY_DATA 37                         // PS2 keyboard data pin
#define KEY_CLOCK 36                        // PS2 keyboard clock pin

// Manual pulse generator configuration
#define PULSE_PER_REVOLUTION 600            // PPR of handwheels

// Motion control constants
#define STEP_TIME_MS 500                    // Time for 1 manual step (ms)
#define DELAY_BETWEEN_STEPS_MS 80          // Time between steps (ms)
#define DIRECTION_SETUP_DELAY_US 5         // Stepper driver direction setup delay
#define STEPPED_ENABLE_DELAY_MS 100        // Delay after stepper enable

// Operation limits
#define DUPR_MAX 254000                     // Maximum pitch (1 inch)
#define STARTS_MAX 124                      // Maximum number of starts
#define PASSES_MAX 999                      // Maximum number of passes
#define SAFE_DISTANCE_DU 5000              // Safe distance for cuts (0.5mm)

// Storage configuration
#define SAVE_DELAY_US 5000000              // Save delay (5 seconds)
#define PREFERENCES_VERSION 1               // Preferences version
#define PREF_NAMESPACE "h5"                // Preferences namespace

// G-code configuration
#define LINEAR_INTERPOLATION_PRECISION 0.1  // Linear interpolation precision
#define GCODE_WAIT_EPSILON_STEPS 10        // G-code wait epsilon
#define SPINDLE_PAUSES_GCODE true          // Pause G-code when spindle stops
#define GCODE_MIN_RPM 30                   // Minimum RPM for G-code
#define GCODE_FEED_DEFAULT_DU_SEC 20000    // Default feed rate
#define GCODE_FEED_MIN_DU_SEC 167          // Minimum feed rate

// Move step constants
#define MOVE_STEP_1 10000                  // 1mm
#define MOVE_STEP_2 1000                   // 0.1mm
#define MOVE_STEP_3 100                    // 0.01mm
#define MOVE_STEP_IMP_1 25400              // 1/10"
#define MOVE_STEP_IMP_2 2540               // 1/100"
#define MOVE_STEP_IMP_3 254                // 1/1000"

// Operation modes
#define MODE_NORMAL 0
#define MODE_ASYNC 2
#define MODE_CONE 3
#define MODE_TURN 4
#define MODE_FACE 5
#define MODE_CUT 6
#define MODE_THREAD 7
#define MODE_ELLIPSE 8
#define MODE_GCODE 9
#define MODE_Y 10

// Measurement systems
#define MEASURE_METRIC 0
#define MEASURE_INCH 1
#define MEASURE_TPI 2

// Emergency stop types
#define ESTOP_NONE 0
#define ESTOP_POS 2
#define ESTOP_MARK_ORIGIN 3
#define ESTOP_ON_OFF 4
#define ESTOP_OFF_MANUAL_MOVE 5

// Timer configuration
#define TIMER_FREQ 1000000                 // 1MHz async timer frequency

// TPI rounding epsilon
#define TPI_ROUND_EPSILON 0.03

// Encoder step calculations
#define ENCODER_STEPS_INT (ENCODER_PPR * 2)
#define ENCODER_STEPS_FLOAT ENCODER_STEPS_INT
#define RPM_BULK ENCODER_STEPS_INT

// Hardware pulse counter units
#define PCNT_UNIT_0 PCNT_UNIT_0
#define PCNT_UNIT_1 PCNT_UNIT_1
#define PCNT_UNIT_2 PCNT_UNIT_2
#define PCNT_UNIT_3 PCNT_UNIT_3

// Button definitions (from original h5.ino)
#define B_LEFT 21                          // Left arrow
#define B_RIGHT 22                         // Right arrow
#define B_UP 23                            // Up arrow
#define B_DOWN 24                          // Down arrow
#define B_MINUS 45                         // Numpad minus
#define B_PLUS 44                          // Numpad plus
#define B_ON 30                            // Enter
#define B_OFF 27                           // ESC
#define B_STOPL 65                         // a - sets left stop
#define B_STOPR 68                         // d - sets right stop
#define B_STOPU 87                         // w - sets forward stop
#define B_STOPD 83                         // s - sets rear stop
#define B_DISPL 12                         // Win - changes info displayed
#define B_STEP 64                          // Tilda - changes distance moved
#define B_SETTINGS 14                      // Context menu
#define B_MEASURE 77                       // m - controls metric/imperial/TPI
#define B_REVERSE 82                       // r - changes pitch sign
#define B_DIAMETER 79                      // o - sets X0
#define B_0 48                             // 0
#define B_1 49                             // 1
#define B_2 50                             // 2
#define B_3 51                             // 3
#define B_4 52                             // 4
#define B_5 53                             // 5
#define B_6 54                             // 6
#define B_7 55                             // 7
#define B_8 56                             // 8
#define B_9 57                             // 9
#define B_BACKSPACE 28                     // Backspace
#define B_MODE_GEARS 97                    // F1 - gearbox mode
#define B_MODE_TURN 98                     // F2 - turn mode
#define B_MODE_FACE 99                     // F3 - face mode
#define B_MODE_CONE 100                    // F4 - cone mode
#define B_MODE_CUT 101                     // F5 - cut mode
#define B_MODE_THREAD 102                  // F6 - thread mode
#define B_MODE_ASYNC 103                   // F7 - async mode
#define B_MODE_ELLIPSE 104                 // F8 - ellipse mode
#define B_MODE_GCODE 105                   // F9 - G-code mode
#define B_MODE_Y 106                       // F10 - Y mode
#define B_X 88                             // x - zeroes X axis
#define B_Z 90                             // z - zeroes Z axis
#define B_X_ENA 67                         // c - enables/disables X axis
#define B_Z_ENA 81                         // q - enables/disables Z axis

// Preferences keys
#define PREF_VERSION "v"
#define PREF_DUPR "d"
#define PREF_POS_Z "zp"
#define PREF_LEFT_STOP_Z "zls"
#define PREF_RIGHT_STOP_Z "zrs"
#define PREF_ORIGIN_POS_Z "zpo"
#define PREF_POS_GLOBAL_Z "zpg"
#define PREF_MOTOR_POS_Z "zpm"
#define PREF_DISABLED_Z "zd"
#define PREF_POS_X "xp"
#define PREF_LEFT_STOP_X "xls"
#define PREF_RIGHT_STOP_X "xrs"
#define PREF_ORIGIN_POS_X "xpo"
#define PREF_POS_GLOBAL_X "xpg"
#define PREF_MOTOR_POS_X "xpm"
#define PREF_DISABLED_X "xd"
#define PREF_POS_Y "y1p"
#define PREF_LEFT_STOP_Y "y1ls"
#define PREF_RIGHT_STOP_Y "y1rs"
#define PREF_ORIGIN_POS_Y "y1po"
#define PREF_POS_GLOBAL_Y "y1pg"
#define PREF_MOTOR_POS_Y "y1pm"
#define PREF_DISABLED_Y "y1d"
#define PREF_SPINDLE_POS "sp"
#define PREF_SPINDLE_POS_AVG "spa"
#define PREF_OUT_OF_SYNC "oos"
#define PREF_SPINDLE_POS_GLOBAL "spg"
#define PREF_SHOW_ANGLE "ang"
#define PREF_SHOW_TACHO "rpm"
#define PREF_STARTS "sta"
#define PREF_MODE "mod"
#define PREF_MEASURE "mea"
#define PREF_CONE_RATIO "cr"
#define PREF_TURN_PASSES "tp"
#define PREF_MOVE_STEP "ms"
#define PREF_AUX_FORWARD "af"

// Hardware utility functions
inline bool isValidPin(int pin) {
    return pin >= 0 && pin <= 48;
}

inline bool isValidMotorSteps(int steps) {
    return steps > 0 && steps <= 10000;
}

inline bool isValidScrewPitch(long pitch) {
    return pitch > 0 && pitch <= 1000000;
}

inline bool isValidSpeed(long speed) {
    return speed > 0 && speed <= 100000;
}

inline bool isValidAcceleration(long accel) {
    return accel > 0 && accel <= 1000000;
}

// Hardware configuration validation
inline bool validateHardwareConfig() {
    return isValidPin(ENC_A) && isValidPin(ENC_B) &&
           isValidPin(Z_STEP) && isValidPin(Z_DIR) && isValidPin(Z_ENA) &&
           isValidPin(X_STEP) && isValidPin(X_DIR) && isValidPin(X_ENA) &&
           isValidMotorSteps(MOTOR_STEPS_Z) && isValidMotorSteps(MOTOR_STEPS_X) &&
           isValidScrewPitch(SCREW_Z_DU) && isValidScrewPitch(SCREW_X_DU) &&
           isValidSpeed(SPEED_START_Z) && isValidSpeed(SPEED_START_X) &&
           isValidAcceleration(ACCELERATION_Z) && isValidAcceleration(ACCELERATION_X);
}