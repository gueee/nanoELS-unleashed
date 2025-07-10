/*
 * hardware_config.h
 * Hardware configuration for nanoELS H5
 * 
 * Defines pin assignments, hardware parameters, and physical constraints
 * for the ESP32-S3 based lathe controller with SN74HCT245N buffers
 */

#pragma once

#include <Arduino.h>

// Hardware version identification
#define HARDWARE_VERSION 5
#define SOFTWARE_VERSION 10

// ESP32-S3 Pin Assignments
// ======================

// Spindle encoder pins (quadrature encoder)
#define SPINDLE_ENC_A    13
#define SPINDLE_ENC_B    14

// Z-axis (main leadscrew) pins
#define Z_ENABLE_PIN     41
#define Z_DIRECTION_PIN  42
#define Z_STEP_PIN       35
#define Z_PULSE_A_PIN    18   // Manual pulse generator
#define Z_PULSE_B_PIN    8

// X-axis (cross-slide) pins
#define X_ENABLE_PIN     16
#define X_DIRECTION_PIN  15
#define X_STEP_PIN       7
#define X_PULSE_A_PIN    47   // Manual pulse generator
#define X_PULSE_B_PIN    21

// Y-axis (optional 4th axis) pins
#define Y_ENABLE_PIN     1
#define Y_DIRECTION_PIN  2
#define Y_STEP_PIN       17
#define Y_PULSE_A_PIN    45   // Manual pulse generator
#define Y_PULSE_B_PIN    48

// PS2 Keyboard pins
#define PS2_DATA_PIN     37
#define PS2_CLOCK_PIN    36

// Emergency stop hardware pin
#define EMERGENCY_STOP_PIN 38

// Hardware limit switch pins
#define Z_LIMIT_MIN_PIN  39
#define Z_LIMIT_MAX_PIN  40
#define X_LIMIT_MIN_PIN  3
#define X_LIMIT_MAX_PIN  4
#define Y_LIMIT_MIN_PIN  5
#define Y_LIMIT_MAX_PIN  6

// Status LED pins
#define STATUS_LED_PIN   46
#define ERROR_LED_PIN    9
#define READY_LED_PIN    10

// Hardware Parameters
// ==================

// Spindle encoder configuration
#define SPINDLE_ENCODER_PPR         1200    // Pulses per revolution
#define SPINDLE_ENCODER_BACKLASH    3       // Backlash compensation pulses
#define SPINDLE_ENCODER_FILTER      1       // Hardware filter (1-1023 clock cycles)

// Stepper motor configuration
#define DEFAULT_MOTOR_STEPS_Z       800     // Full steps per revolution
#define DEFAULT_MOTOR_STEPS_X       800
#define DEFAULT_MOTOR_STEPS_Y       800

// Lead screw configuration (in deci-microns, 10^-7 meters)
#define DEFAULT_SCREW_PITCH_Z       40000   // 4mm lead screw
#define DEFAULT_SCREW_PITCH_X       40000   // 4mm lead screw
#define DEFAULT_SCREW_PITCH_Y       20000   // 2mm lead screw

// Motion limits (in millimeters)
#define DEFAULT_MAX_TRAVEL_Z        300     // 300mm travel
#define DEFAULT_MAX_TRAVEL_X        100     // 100mm travel
#define DEFAULT_MAX_TRAVEL_Y        360     // 360 degrees for rotary axis

// Speed and acceleration limits
#define DEFAULT_SPEED_START_Z       800     // Initial speed (steps/sec)
#define DEFAULT_SPEED_MAX_Z         6400    // Maximum speed (steps/sec)
#define DEFAULT_ACCELERATION_Z      20000   // Acceleration (steps/sec²)

#define DEFAULT_SPEED_START_X       800
#define DEFAULT_SPEED_MAX_X         6400
#define DEFAULT_ACCELERATION_X      20000

#define DEFAULT_SPEED_START_Y       1600
#define DEFAULT_SPEED_MAX_Y         3200
#define DEFAULT_ACCELERATION_Y      16000

// Timing constraints (microseconds)
#define DIRECTION_SETUP_DELAY_US    5       // Direction signal setup time
#define STEP_PULSE_WIDTH_US         10      // Minimum step pulse width
#define ENABLE_DELAY_MS             100     // Delay after enable before stepping

// Safety parameters
#define EMERGENCY_STOP_DEBOUNCE_MS  10      // Emergency stop debounce time
#define WATCHDOG_TIMEOUT_MS         5000    // Watchdog timeout
#define POSITION_TOLERANCE_STEPS    10      // Position tolerance for limits

// Hardware pulse counter configuration
#define PCNT_FILTER_LENGTH          1       // Filter length for pulse counters
#define PCNT_LIMIT_HIGH             31000   // High limit for pulse counter
#define PCNT_LIMIT_LOW              -31000  // Low limit for pulse counter

// Communication configuration
#define SERIAL_BAUD_RATE            115200  // Main serial port
#define DISPLAY_BAUD_RATE           115200  // Nextion display
#define DISPLAY_SERIAL_PORT         Serial1 // Hardware serial for display

// WiFi configuration
#define WIFI_CONNECT_TIMEOUT_MS     10000   // WiFi connection timeout
#define WEBSOCKET_PORT              81      // WebSocket server port
#define HTTP_PORT                   80      // HTTP server port

// Buffer sizes
#define WEBSOCKET_BUFFER_SIZE       100000  // WebSocket buffer size
#define GCODE_BUFFER_SIZE           50000   // G-code buffer size
#define KEYBOARD_BUFFER_SIZE        32      // Keyboard input buffer

// Memory configuration
#define TASK_STACK_SIZE_MOTION      10000   // Motion control task stack
#define TASK_STACK_SIZE_SAFETY      4000    // Safety monitoring task stack
#define TASK_STACK_SIZE_DISPLAY     8000    // Display task stack
#define TASK_STACK_SIZE_COMM        8000    // Communication task stack

// Hardware feature flags
#define HARDWARE_HAS_EMERGENCY_STOP true    // Hardware emergency stop present
#define HARDWARE_HAS_LIMIT_SWITCHES true    // Hardware limit switches present
#define HARDWARE_HAS_STATUS_LEDS    true    // Status LEDs present
#define HARDWARE_HAS_Y_AXIS         false   // Y-axis hardware present
#define HARDWARE_HAS_MPG            true    // Manual pulse generators present

// Validation macros
#define VALIDATE_PIN(pin) ((pin >= 0) && (pin <= 48))
#define VALIDATE_SPEED(speed) ((speed > 0) && (speed <= 50000))
#define VALIDATE_ACCELERATION(accel) ((accel > 0) && (accel <= 100000))

// Pin state macros for step/direction control
#define STEP_HIGH(pin)      digitalWrite(pin, HIGH)
#define STEP_LOW(pin)       digitalWrite(pin, LOW)
#define DIR_FORWARD(pin)    digitalWrite(pin, LOW)
#define DIR_REVERSE(pin)    digitalWrite(pin, HIGH)
#define ENABLE_ON(pin)      digitalWrite(pin, LOW)
#define ENABLE_OFF(pin)     digitalWrite(pin, HIGH)

// Hardware validation functions
inline bool validateHardwareConfiguration() {
    // Validate pin assignments
    if (!VALIDATE_PIN(SPINDLE_ENC_A) || !VALIDATE_PIN(SPINDLE_ENC_B)) return false;
    if (!VALIDATE_PIN(Z_STEP_PIN) || !VALIDATE_PIN(Z_DIRECTION_PIN) || !VALIDATE_PIN(Z_ENABLE_PIN)) return false;
    if (!VALIDATE_PIN(X_STEP_PIN) || !VALIDATE_PIN(X_DIRECTION_PIN) || !VALIDATE_PIN(X_ENABLE_PIN)) return false;
    if (!VALIDATE_PIN(PS2_DATA_PIN) || !VALIDATE_PIN(PS2_CLOCK_PIN)) return false;
    if (!VALIDATE_PIN(EMERGENCY_STOP_PIN)) return false;
    
    // Validate speed and acceleration parameters
    if (!VALIDATE_SPEED(DEFAULT_SPEED_MAX_Z) || !VALIDATE_SPEED(DEFAULT_SPEED_MAX_X)) return false;
    if (!VALIDATE_ACCELERATION(DEFAULT_ACCELERATION_Z) || !VALIDATE_ACCELERATION(DEFAULT_ACCELERATION_X)) return false;
    
    return true;
}

// Hardware initialization function
inline void initializeHardwarePins() {
    // Configure stepper motor pins
    pinMode(Z_STEP_PIN, OUTPUT);
    pinMode(Z_DIRECTION_PIN, OUTPUT);
    pinMode(Z_ENABLE_PIN, OUTPUT);
    
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIRECTION_PIN, OUTPUT);
    pinMode(X_ENABLE_PIN, OUTPUT);
    
    if (HARDWARE_HAS_Y_AXIS) {
        pinMode(Y_STEP_PIN, OUTPUT);
        pinMode(Y_DIRECTION_PIN, OUTPUT);
        pinMode(Y_ENABLE_PIN, OUTPUT);
    }
    
    // Configure encoder pins
    pinMode(SPINDLE_ENC_A, INPUT_PULLUP);
    pinMode(SPINDLE_ENC_B, INPUT_PULLUP);
    
    // Configure MPG pins
    pinMode(Z_PULSE_A_PIN, INPUT_PULLUP);
    pinMode(Z_PULSE_B_PIN, INPUT_PULLUP);
    pinMode(X_PULSE_A_PIN, INPUT_PULLUP);
    pinMode(X_PULSE_B_PIN, INPUT_PULLUP);
    
    if (HARDWARE_HAS_Y_AXIS) {
        pinMode(Y_PULSE_A_PIN, INPUT_PULLUP);
        pinMode(Y_PULSE_B_PIN, INPUT_PULLUP);
    }
    
    // Configure safety pins
    if (HARDWARE_HAS_EMERGENCY_STOP) {
        pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    }
    
    if (HARDWARE_HAS_LIMIT_SWITCHES) {
        pinMode(Z_LIMIT_MIN_PIN, INPUT_PULLUP);
        pinMode(Z_LIMIT_MAX_PIN, INPUT_PULLUP);
        pinMode(X_LIMIT_MIN_PIN, INPUT_PULLUP);
        pinMode(X_LIMIT_MAX_PIN, INPUT_PULLUP);
        
        if (HARDWARE_HAS_Y_AXIS) {
            pinMode(Y_LIMIT_MIN_PIN, INPUT_PULLUP);
            pinMode(Y_LIMIT_MAX_PIN, INPUT_PULLUP);
        }
    }
    
    // Configure status LEDs
    if (HARDWARE_HAS_STATUS_LEDS) {
        pinMode(STATUS_LED_PIN, OUTPUT);
        pinMode(ERROR_LED_PIN, OUTPUT);
        pinMode(READY_LED_PIN, OUTPUT);
        
        // Initialize LED states
        digitalWrite(STATUS_LED_PIN, LOW);
        digitalWrite(ERROR_LED_PIN, LOW);
        digitalWrite(READY_LED_PIN, LOW);
    }
    
    // Initialize stepper outputs to safe state
    STEP_LOW(Z_STEP_PIN);
    STEP_LOW(X_STEP_PIN);
    DIR_FORWARD(Z_DIRECTION_PIN);
    DIR_FORWARD(X_DIRECTION_PIN);
    ENABLE_OFF(Z_ENABLE_PIN);
    ENABLE_OFF(X_ENABLE_PIN);
    
    if (HARDWARE_HAS_Y_AXIS) {
        STEP_LOW(Y_STEP_PIN);
        DIR_FORWARD(Y_DIRECTION_PIN);
        ENABLE_OFF(Y_ENABLE_PIN);
    }
}