/*
 * safety_config.h
 * Safety configuration and constants for nanoELS H5
 * 
 * Defines safety limits, emergency stop types, and safety-critical
 * parameters for the lathe controller system
 */

#pragma once

#include <Arduino.h>

// Emergency Stop Types
// ===================
enum EmergencyStopType {
    ESTOP_NONE = 0,                 // Normal operation
    ESTOP_HARDWARE_BUTTON = 1,      // Hardware emergency stop button pressed
    ESTOP_POS_LIMIT = 2,            // Position outside machine limits
    ESTOP_HARDWARE_FAILURE = 3,     // Hardware failure detected
    ESTOP_COMMUNICATION_LOSS = 4,   // Communication failure
    ESTOP_ENCODER_FAILURE = 5,      // Spindle encoder failure
    ESTOP_MOTOR_STALL = 6,          // Motor stall detected
    ESTOP_THERMAL_PROTECTION = 7,   // Thermal protection triggered
    ESTOP_POWER_FAILURE = 8,        // Power supply failure
    ESTOP_SOFTWARE_ERROR = 9,       // Software error condition
    ESTOP_SAFETY_VIOLATION = 10,    // Safety system violation
    ESTOP_USER_REQUESTED = 11,      // User-requested emergency stop
    ESTOP_LIMIT_SWITCH = 12,        // Hardware limit switch triggered
    ESTOP_WATCHDOG_TIMEOUT = 13,    // Watchdog timer timeout
    ESTOP_INVALID_COMMAND = 14      // Invalid command received
};

// Safety System States
// ===================
enum SafetySystemState {
    SAFETY_STATE_INIT = 0,          // Initializing safety systems
    SAFETY_STATE_CHECKING = 1,      // Performing safety checks
    SAFETY_STATE_SAFE = 2,          // All systems safe
    SAFETY_STATE_WARNING = 3,       // Warning condition present
    SAFETY_STATE_EMERGENCY = 4,     // Emergency stop active
    SAFETY_STATE_FAULT = 5,         // System fault requiring reset
    SAFETY_STATE_MAINTENANCE = 6    // Maintenance mode
};

// Motion Safety Limits
// ====================

// Maximum safe speeds (steps per second)
#define MAX_SAFE_SPEED_Z            10000   // Z-axis maximum safe speed
#define MAX_SAFE_SPEED_X            10000   // X-axis maximum safe speed
#define MAX_SAFE_SPEED_Y            5000    // Y-axis maximum safe speed

// Maximum safe accelerations (steps per second squared)
#define MAX_SAFE_ACCELERATION_Z     50000   // Z-axis maximum safe acceleration
#define MAX_SAFE_ACCELERATION_X     50000   // X-axis maximum safe acceleration
#define MAX_SAFE_ACCELERATION_Y     25000   // Y-axis maximum safe acceleration

// Position safety limits (in steps from home)
#define MAX_SAFE_POSITION_Z         240000  // 300mm * 800 steps/mm
#define MAX_SAFE_POSITION_X         80000   // 100mm * 800 steps/mm
#define MAX_SAFE_POSITION_Y         288000  // 360° * 800 steps/degree

// Minimum safe distances (in steps)
#define MIN_SAFE_DISTANCE_STEPS     400     // 0.5mm minimum safe distance
#define RAPID_APPROACH_DISTANCE     4000    // 5mm rapid approach distance

// Timing Safety Constraints
// =========================

// Step pulse timing (microseconds)
#define MIN_STEP_PULSE_WIDTH_US     10      // Minimum step pulse width
#define MIN_STEP_INTERVAL_US        50      // Minimum time between steps
#define MIN_DIRECTION_SETUP_US      5       // Minimum direction setup time

// Safety monitoring intervals (milliseconds)
#define SAFETY_CHECK_INTERVAL_MS    10      // Safety system check interval
#define WATCHDOG_FEED_INTERVAL_MS   100     // Watchdog feed interval
#define POSITION_CHECK_INTERVAL_MS  5       // Position limit check interval

// Emergency stop response times (microseconds)
#define EMERGENCY_STOP_RESPONSE_US  100     // Maximum emergency stop response time
#define MOTION_HALT_TIMEOUT_US      1000    // Maximum time to halt motion

// Hardware Safety Parameters
// ==========================

// Debounce times (milliseconds)
#define ESTOP_DEBOUNCE_TIME_MS      10      // Emergency stop button debounce
#define LIMIT_SWITCH_DEBOUNCE_MS    5       // Limit switch debounce
#define ENCODER_FAULT_DEBOUNCE_MS   100     // Encoder fault detection debounce

// Timeout values (milliseconds)
#define WATCHDOG_TIMEOUT_MS         5000    // Hardware watchdog timeout
#define MOTION_TIMEOUT_MS           30000   // Motion command timeout
#define COMMUNICATION_TIMEOUT_MS    1000    // Communication timeout

// Hardware fault detection thresholds
#define ENCODER_FAULT_COUNT_THRESHOLD   100     // Encoder error count threshold
#define MOTOR_STALL_DETECTION_TIME_MS   5000    // Motor stall detection time
#define THERMAL_WARNING_TEMP_C          80      // Thermal warning temperature
#define THERMAL_SHUTDOWN_TEMP_C         95      // Thermal shutdown temperature

// Position tolerance and validation
#define POSITION_TOLERANCE_STEPS    10      // Position tolerance in steps
#define VELOCITY_TOLERANCE_PERCENT  5       // Velocity tolerance percentage
#define ACCELERATION_TOLERANCE_PERCENT 10   // Acceleration tolerance percentage

// Safety Validation Macros
// ========================

// Validate speed is within safe limits
#define IS_SAFE_SPEED_Z(speed) ((speed >= 0) && (speed <= MAX_SAFE_SPEED_Z))
#define IS_SAFE_SPEED_X(speed) ((speed >= 0) && (speed <= MAX_SAFE_SPEED_X))
#define IS_SAFE_SPEED_Y(speed) ((speed >= 0) && (speed <= MAX_SAFE_SPEED_Y))

// Validate acceleration is within safe limits
#define IS_SAFE_ACCEL_Z(accel) ((accel >= 0) && (accel <= MAX_SAFE_ACCELERATION_Z))
#define IS_SAFE_ACCEL_X(accel) ((accel >= 0) && (accel <= MAX_SAFE_ACCELERATION_X))
#define IS_SAFE_ACCEL_Y(accel) ((accel >= 0) && (accel <= MAX_SAFE_ACCELERATION_Y))

// Validate position is within safe limits
#define IS_SAFE_POSITION_Z(pos) ((abs(pos) <= MAX_SAFE_POSITION_Z))
#define IS_SAFE_POSITION_X(pos) ((abs(pos) <= MAX_SAFE_POSITION_X))
#define IS_SAFE_POSITION_Y(pos) ((abs(pos) <= MAX_SAFE_POSITION_Y))

// Check if distance is safe for rapid movement
#define IS_SAFE_RAPID_DISTANCE(dist) ((abs(dist) >= MIN_SAFE_DISTANCE_STEPS))

// Safety State Transition Validation
// ==================================

// Validate state transitions are allowed
inline bool isValidStateTransition(SafetySystemState from, SafetySystemState to) {
    switch (from) {
        case SAFETY_STATE_INIT:
            return (to == SAFETY_STATE_CHECKING || to == SAFETY_STATE_FAULT);
            
        case SAFETY_STATE_CHECKING:
            return (to == SAFETY_STATE_SAFE || to == SAFETY_STATE_WARNING || 
                   to == SAFETY_STATE_EMERGENCY || to == SAFETY_STATE_FAULT);
            
        case SAFETY_STATE_SAFE:
            return (to == SAFETY_STATE_WARNING || to == SAFETY_STATE_EMERGENCY || 
                   to == SAFETY_STATE_MAINTENANCE);
            
        case SAFETY_STATE_WARNING:
            return (to == SAFETY_STATE_SAFE || to == SAFETY_STATE_EMERGENCY);
            
        case SAFETY_STATE_EMERGENCY:
            return (to == SAFETY_STATE_CHECKING || to == SAFETY_STATE_FAULT);
            
        case SAFETY_STATE_FAULT:
            return (to == SAFETY_STATE_INIT);
            
        case SAFETY_STATE_MAINTENANCE:
            return (to == SAFETY_STATE_CHECKING);
            
        default:
            return false;
    }
}

// Emergency Stop Priority Levels
// ==============================
enum EmergencyStopPriority {
    ESTOP_PRIORITY_LOW = 1,         // Low priority - can be overridden
    ESTOP_PRIORITY_MEDIUM = 2,      // Medium priority - requires acknowledgment
    ESTOP_PRIORITY_HIGH = 3,        // High priority - immediate action required
    ESTOP_PRIORITY_CRITICAL = 4     // Critical - hardware level, cannot be overridden
};

// Get emergency stop priority
inline EmergencyStopPriority getEmergencyStopPriority(EmergencyStopType type) {
    switch (type) {
        case ESTOP_HARDWARE_BUTTON:
        case ESTOP_LIMIT_SWITCH:
        case ESTOP_POWER_FAILURE:
            return ESTOP_PRIORITY_CRITICAL;
            
        case ESTOP_POS_LIMIT:
        case ESTOP_MOTOR_STALL:
        case ESTOP_THERMAL_PROTECTION:
        case ESTOP_WATCHDOG_TIMEOUT:
            return ESTOP_PRIORITY_HIGH;
            
        case ESTOP_HARDWARE_FAILURE:
        case ESTOP_ENCODER_FAILURE:
        case ESTOP_SAFETY_VIOLATION:
            return ESTOP_PRIORITY_MEDIUM;
            
        case ESTOP_COMMUNICATION_LOSS:
        case ESTOP_SOFTWARE_ERROR:
        case ESTOP_USER_REQUESTED:
        case ESTOP_INVALID_COMMAND:
            return ESTOP_PRIORITY_LOW;
            
        default:
            return ESTOP_PRIORITY_MEDIUM;
    }
}

// Safety Configuration Validation
// ===============================

// Validate all safety configuration parameters
inline bool validateSafetyConfiguration() {
    // Check timing constraints
    if (MIN_STEP_PULSE_WIDTH_US < 1 || MIN_STEP_PULSE_WIDTH_US > 1000) return false;
    if (MIN_DIRECTION_SETUP_US < 1 || MIN_DIRECTION_SETUP_US > 100) return false;
    if (EMERGENCY_STOP_RESPONSE_US < 10 || EMERGENCY_STOP_RESPONSE_US > 10000) return false;
    
    // Check safety limits
    if (MAX_SAFE_SPEED_Z <= 0 || MAX_SAFE_SPEED_Z > 100000) return false;
    if (MAX_SAFE_SPEED_X <= 0 || MAX_SAFE_SPEED_X > 100000) return false;
    if (MAX_SAFE_ACCELERATION_Z <= 0 || MAX_SAFE_ACCELERATION_Z > 1000000) return false;
    if (MAX_SAFE_ACCELERATION_X <= 0 || MAX_SAFE_ACCELERATION_X > 1000000) return false;
    
    // Check position limits
    if (MAX_SAFE_POSITION_Z <= 0 || MAX_SAFE_POSITION_X <= 0) return false;
    if (MIN_SAFE_DISTANCE_STEPS <= 0 || MIN_SAFE_DISTANCE_STEPS > 10000) return false;
    
    // Check timeout values
    if (WATCHDOG_TIMEOUT_MS < 1000 || WATCHDOG_TIMEOUT_MS > 60000) return false;
    if (MOTION_TIMEOUT_MS < 1000 || MOTION_TIMEOUT_MS > 300000) return false;
    
    return true;
}

// Error Messages for Safety Violations
// ====================================

inline const char* getEmergencyStopMessage(EmergencyStopType type) {
    switch (type) {
        case ESTOP_NONE:
            return "System operational";
        case ESTOP_HARDWARE_BUTTON:
            return "Hardware emergency stop button pressed";
        case ESTOP_POS_LIMIT:
            return "Position outside machine limits";
        case ESTOP_HARDWARE_FAILURE:
            return "Hardware failure detected";
        case ESTOP_COMMUNICATION_LOSS:
            return "Communication failure";
        case ESTOP_ENCODER_FAILURE:
            return "Spindle encoder failure";
        case ESTOP_MOTOR_STALL:
            return "Motor stall detected";
        case ESTOP_THERMAL_PROTECTION:
            return "Thermal protection triggered";
        case ESTOP_POWER_FAILURE:
            return "Power supply failure";
        case ESTOP_SOFTWARE_ERROR:
            return "Software error condition";
        case ESTOP_SAFETY_VIOLATION:
            return "Safety system violation";
        case ESTOP_USER_REQUESTED:
            return "User-requested emergency stop";
        case ESTOP_LIMIT_SWITCH:
            return "Hardware limit switch triggered";
        case ESTOP_WATCHDOG_TIMEOUT:
            return "Watchdog timer timeout";
        case ESTOP_INVALID_COMMAND:
            return "Invalid command received";
        default:
            return "Unknown emergency stop condition";
    }
}