/*
 * safety_config.h
 * Safety Configuration for nanoELS H5
 * 
 * Contains safety-related constants, thresholds, and configuration
 * for the lathe controller safety system.
 */

#pragma once

#include <Arduino.h>

// Safety system states
enum SafetySystemState {
    SAFETY_INITIALIZING = 0,
    SAFETY_READY = 1,
    SAFETY_WARNING = 2,
    SAFETY_ERROR = 3,
    SAFETY_EMERGENCY = 4,
    SAFETY_FAULT = 5
};

// Emergency stop types
enum EmergencyStopType {
    ESTOP_NONE = 0,
    ESTOP_USER_REQUESTED = 1,
    ESTOP_SAFETY_VIOLATION = 2,
    ESTOP_POS_LIMIT = 3,
    ESTOP_HARDWARE_FAILURE = 4,
    ESTOP_WATCHDOG_TIMEOUT = 5,
    ESTOP_COMMUNICATION_FAILURE = 6,
    ESTOP_ENCODER_FAULT = 7,
    ESTOP_MOTOR_STALL = 8,
    ESTOP_TEMPERATURE_FAULT = 9,
    ESTOP_POWER_FAULT = 10
};

// Emergency stop priorities
enum EmergencyStopPriority {
    ESTOP_PRIORITY_LOW = 0,
    ESTOP_PRIORITY_MEDIUM = 1,
    ESTOP_PRIORITY_HIGH = 2,
    ESTOP_PRIORITY_CRITICAL = 3
};

// Safety timeouts and thresholds
#define SAFETY_WATCHDOG_TIMEOUT_MS 5000        // Watchdog timeout (5 seconds)
#define SAFETY_EMERGENCY_DEBOUNCE_MS 10         // Emergency stop debounce time
#define SAFETY_LIMIT_DEBOUNCE_MS 5              // Limit switch debounce time
#define SAFETY_ENCODER_TIMEOUT_MS 1000          // Encoder signal timeout
#define SAFETY_MOTOR_STALL_TIMEOUT_MS 5000      // Motor stall detection timeout
#define SAFETY_TEMPERATURE_CHECK_INTERVAL_MS 1000 // Temperature check interval
#define SAFETY_POWER_CHECK_INTERVAL_MS 500      // Power check interval
#define SAFETY_COMMUNICATION_TIMEOUT_MS 10000   // Communication timeout

// Position and motion safety limits
#define SAFETY_MAX_SPEED_Z 10000                // Maximum Z axis speed (steps/sec)
#define SAFETY_MAX_SPEED_X 10000                // Maximum X axis speed (steps/sec)
#define SAFETY_MAX_SPEED_Y 5000                 // Maximum Y axis speed (steps/sec)
#define SAFETY_MAX_ACCELERATION_Z 50000         // Maximum Z axis acceleration
#define SAFETY_MAX_ACCELERATION_X 50000         // Maximum X axis acceleration
#define SAFETY_MAX_ACCELERATION_Y 25000         // Maximum Y axis acceleration
#define SAFETY_POSITION_TOLERANCE_STEPS 10      // Position tolerance for limits
#define SAFETY_VELOCITY_TOLERANCE 0.1           // Velocity tolerance (%)

// Temperature safety limits
#define SAFETY_MAX_TEMPERATURE_MOTOR 80         // Maximum motor temperature (°C)
#define SAFETY_MAX_TEMPERATURE_DRIVER 70        // Maximum driver temperature (°C)
#define SAFETY_MAX_TEMPERATURE_CONTROLLER 60    // Maximum controller temperature (°C)
#define SAFETY_TEMPERATURE_WARNING_THRESHOLD 0.8 // Temperature warning threshold (%)

// Power safety limits
#define SAFETY_MIN_VOLTAGE 10.0                 // Minimum voltage (V)
#define SAFETY_MAX_VOLTAGE 15.0                 // Maximum voltage (V)
#define SAFETY_MAX_CURRENT_Z 5.0                // Maximum Z axis current (A)
#define SAFETY_MAX_CURRENT_X 5.0                // Maximum X axis current (A)
#define SAFETY_MAX_CURRENT_Y 3.0                // Maximum Y axis current (A)

// Encoder safety limits
#define SAFETY_MIN_ENCODER_RPM 1                // Minimum encoder RPM
#define SAFETY_MAX_ENCODER_RPM 3000             // Maximum encoder RPM
#define SAFETY_ENCODER_SIGNAL_MIN_INTERVAL_US 100 // Minimum encoder signal interval
#define SAFETY_ENCODER_SIGNAL_MAX_INTERVAL_US 1000000 // Maximum encoder signal interval

// Communication safety limits
#define SAFETY_MAX_COMMUNICATION_DELAY_MS 1000  // Maximum communication delay
#define SAFETY_MIN_COMMUNICATION_INTERVAL_MS 10 // Minimum communication interval

// Safety fault counters
#define SAFETY_MAX_FAULTS_BEFORE_ESTOP 3        // Maximum faults before emergency stop
#define SAFETY_FAULT_RESET_TIMEOUT_MS 60000     // Fault reset timeout (1 minute)
#define SAFETY_WARNING_RESET_TIMEOUT_MS 30000   // Warning reset timeout (30 seconds)

// Safety override settings
#define SAFETY_OVERRIDE_MAX_DURATION_MS 30000   // Maximum safety override duration
#define SAFETY_OVERRIDE_REQUIRES_CONFIRMATION true // Require confirmation for override

// Hardware safety pins
#define SAFETY_EMERGENCY_STOP_PIN -1             // Emergency stop pin (not used in H5)
#define SAFETY_LIMIT_Z_MIN_PIN -1               // Z axis minimum limit pin
#define SAFETY_LIMIT_Z_MAX_PIN -1               // Z axis maximum limit pin
#define SAFETY_LIMIT_X_MIN_PIN -1               // X axis minimum limit pin
#define SAFETY_LIMIT_X_MAX_PIN -1               // X axis maximum limit pin
#define SAFETY_LIMIT_Y_MIN_PIN -1               // Y axis minimum limit pin
#define SAFETY_LIMIT_Y_MAX_PIN -1               // Y axis maximum limit pin

// Safety status LED pins
#define SAFETY_STATUS_LED_PIN -1                 // Status LED pin
#define SAFETY_ERROR_LED_PIN -1                  // Error LED pin
#define SAFETY_READY_LED_PIN -1                  // Ready LED pin

// Safety validation functions
inline bool isValidTemperature(float temp) {
    return temp >= -40.0f && temp <= 150.0f;
}

inline bool isValidVoltage(float voltage) {
    return voltage >= 0.0f && voltage <= 20.0f;
}

inline bool isValidCurrent(float current) {
    return current >= 0.0f && current <= 10.0f;
}

inline bool isValidRPM(float rpm) {
    return rpm >= 0.0f && rpm <= 10000.0f;
}

inline bool isValidSpeed(long speed) {
    return speed >= 0 && speed <= 50000;
}

inline bool isValidAcceleration(long accel) {
    return accel >= 0 && accel <= 100000;
}

// Safety check macros
#define SAFETY_CHECK_TEMPERATURE(temp) do { \
    if (!isValidTemperature(temp)) { \
        return false; \
    } \
} while(0)

#define SAFETY_CHECK_VOLTAGE(voltage) do { \
    if (!isValidVoltage(voltage)) { \
        return false; \
    } \
} while(0)

#define SAFETY_CHECK_CURRENT(current) do { \
    if (!isValidCurrent(current)) { \
        return false; \
    } \
} while(0)

#define SAFETY_CHECK_RPM(rpm) do { \
    if (!isValidRPM(rpm)) { \
        return false; \
    } \
} while(0)

#define SAFETY_CHECK_SPEED(speed) do { \
    if (!isValidSpeed(speed)) { \
        return false; \
    } \
} while(0)

#define SAFETY_CHECK_ACCELERATION(accel) do { \
    if (!isValidAcceleration(accel)) { \
        return false; \
    } \
} while(0)

// Safety state validation
inline bool isValidSafetyState(SafetySystemState state) {
    return state >= SAFETY_INITIALIZING && state <= SAFETY_FAULT;
}

inline bool isSafetyCritical(SafetySystemState state) {
    return state == SAFETY_EMERGENCY || state == SAFETY_FAULT;
}

inline bool isSafetyWarning(SafetySystemState state) {
    return state == SAFETY_WARNING;
}

inline bool isSafetyReady(SafetySystemState state) {
    return state == SAFETY_READY;
}

// Emergency stop validation
inline bool isValidEmergencyStop(EmergencyStopType type) {
    return type >= ESTOP_NONE && type <= ESTOP_POWER_FAULT;
}

inline bool isEmergencyStopActive(EmergencyStopType type) {
    return type != ESTOP_NONE;
}

inline bool isEmergencyStopCritical(EmergencyStopType type) {
    return type == ESTOP_SAFETY_VIOLATION || 
           type == ESTOP_HARDWARE_FAILURE || 
           type == ESTOP_WATCHDOG_TIMEOUT ||
           type == ESTOP_POWER_FAULT;
}

// Safety priority validation
inline bool isValidEmergencyPriority(EmergencyStopPriority priority) {
    return priority >= ESTOP_PRIORITY_LOW && priority <= ESTOP_PRIORITY_CRITICAL;
}

inline bool isHighPriorityEmergency(EmergencyStopPriority priority) {
    return priority >= ESTOP_PRIORITY_HIGH;
}

// Safety timeout validation
inline bool isValidSafetyTimeout(unsigned long timeout) {
    return timeout > 0 && timeout <= 60000; // 1 minute maximum
}

// Safety fault counting
inline bool shouldTriggerEmergencyStop(int faultCount) {
    return faultCount >= SAFETY_MAX_FAULTS_BEFORE_ESTOP;
}

inline bool shouldResetFaults(unsigned long lastFaultTime) {
    return (millis() - lastFaultTime) > SAFETY_FAULT_RESET_TIMEOUT_MS;
}

// Safety override validation
inline bool isValidSafetyOverride(unsigned long duration) {
    return duration > 0 && duration <= SAFETY_OVERRIDE_MAX_DURATION_MS;
}

// Safety communication validation
inline bool isValidCommunicationDelay(unsigned long delay) {
    return delay <= SAFETY_MAX_COMMUNICATION_DELAY_MS;
}

inline bool isValidCommunicationInterval(unsigned long interval) {
    return interval >= SAFETY_MIN_COMMUNICATION_INTERVAL_MS;
}