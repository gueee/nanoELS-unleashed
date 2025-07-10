/*
 * axis_controller.h
 * Axis Controller Class for nanoELS H5
 * 
 * Controls individual stepper motor axes with safety, precision timing,
 * and real-time motion control capabilities.
 */

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/pcnt.h>
#include "config/hardware_config.h"
#include "config/safety_config.h"

// Axis configuration structure
struct AxisConfig {
    char name;                      // Axis identifier ('X', 'Y', 'Z')
    bool active;                    // Whether axis is enabled
    bool rotational;                // True for rotational, false for linear
    
    // Hardware pins
    int stepPin;
    int directionPin;
    int enablePin;
    int pulseAPin;                  // MPG encoder A
    int pulseBPin;                  // MPG encoder B
    int limitMinPin;                // Minimum limit switch
    int limitMaxPin;                // Maximum limit switch
    
    // Mechanical parameters
    float motorSteps;               // Steps per revolution
    float screwPitch;               // Lead screw pitch (deci-microns)
    float backlashSteps;            // Backlash compensation steps
    
    // Motion parameters
    long speedStart;                // Starting speed (steps/sec)
    long speedMax;                  // Maximum speed (steps/sec)
    long acceleration;              // Acceleration (steps/sec²)
    long maxTravelMm;               // Maximum travel distance (mm)
    
    // Configuration flags
    bool invertDirection;           // Invert direction signal
    bool invertEnable;              // Invert enable signal
    bool needsRest;                 // Whether motor needs holding torque
};

// Motion state enumeration
enum MotionState {
    MOTION_IDLE = 0,
    MOTION_ACCELERATING = 1,
    MOTION_CONSTANT = 2,
    MOTION_DECELERATING = 3,
    MOTION_STOPPING = 4,
    MOTION_ERROR = 5
};

// Axis Controller Class
class AxisController {
private:
    // Configuration
    AxisConfig config;
    
    // Position tracking
    volatile long currentPosition;      // Current position in steps
    volatile long targetPosition;       // Target position in steps
    volatile long motorPosition;        // Motor position (with backlash)
    volatile float fractionalPosition;  // Sub-step positioning
    
    // Motion control
    volatile long currentSpeed;         // Current speed (steps/sec)
    volatile long targetSpeed;          // Target speed (steps/sec)
    volatile MotionState motionState;   // Current motion state
    volatile bool directionForward;     // Current direction
    volatile unsigned long lastStepTime; // Last step pulse time (microseconds)
    
    // Limits and safety
    volatile long leftStop;             // Left/minimum position limit
    volatile long rightStop;            // Right/maximum position limit
    volatile long originPosition;       // Origin/home position
    volatile bool limitsEnabled;        // Whether soft limits are active
    volatile bool emergencyStopActive; // Emergency stop state
    
    // Hardware state
    volatile bool enabled;              // Axis enable state
    volatile bool stepState;            // Current step pin state
    volatile bool directionState;       // Current direction pin state
    volatile bool enableState;          // Current enable pin state
    
    // Manual pulse generator
    pcnt_unit_t mpgUnit;               // Hardware pulse counter unit
    volatile long mpgPosition;          // MPG position
    volatile bool mpgEnabled;           // MPG control enabled
    
    // Thread safety
    SemaphoreHandle_t axisMutex;
    
    // Private methods
    void updateDirection();
    void generateStepPulse();
    void applyBacklashCompensation();
    void calculateMotionProfile();
    bool checkPositionLimits(long position);
    void updateMotionState();
    void handleEmergencyCondition();
    
    // Hardware control
    void setStepPin(bool state);
    void setDirectionPin(bool direction);
    void setEnablePin(bool enable);
    void waitForDirectionSetup();
    
    // MPG handling
    void initializeMPG();
    void processMPGInput();
    
public:
    AxisController(char axisName, const AxisConfig& configuration);
    ~AxisController();
    
    // Initialization
    bool initializeAxis();
    bool validateConfiguration();
    void resetPosition();
    void setOrigin();
    
    // Configuration management
    bool updateConfiguration(const AxisConfig& newConfig);
    AxisConfig getConfiguration() const;
    void applyConfigurationChanges();
    
    // Motion control
    void setTargetPosition(long position);
    void moveToPosition(long position, long speed = 0);
    void moveRelative(long distance, long speed = 0);
    void jogContinuous(bool forward, long speed);
    void stopMotion(bool immediate = false);
    
    // Position and status queries
    long getCurrentPosition() const;
    long getTargetPosition() const;
    long getMotorPosition() const;
    float getFractionalPosition() const;
    MotionState getMotionState() const;
    long getCurrentSpeed() const;
    
    // Limits and safety
    void setLeftStop(long position);
    void setRightStop(long position);
    long getLeftStop() const;
    long getRightStop() const;
    void enableLimits(bool enable);
    bool areLimitsEnabled() const;
    bool isWithinLimits(long position) const;
    
    // Axis control
    void enableAxis(bool enable);
    bool isEnabled() const;
    bool isMoving() const;
    bool isAtTarget() const;
    bool hasError() const;
    
    // Emergency handling
    void handleEmergencyStop();
    void clearEmergencyStop();
    bool isEmergencyStopActive() const;
    
    // Manual pulse generator
    void enableMPG(bool enable);
    bool isMPGEnabled() const;
    long getMPGPosition() const;
    void resetMPGPosition();
    
    // Motion profile and timing
    void updateMotionProfile();
    void executeStep();
    unsigned long getNextStepTime() const;
    bool isStepDue() const;
    
    // Calibration and homing
    bool performHoming();
    bool calibrateBacklash();
    void setPositionFromEncoder(long encoderPosition);
    
    // Diagnostics and monitoring
    unsigned long getStepCount() const;
    unsigned long getLastStepTime() const;
    float getPositionError() const;
    bool performSelfTest();
    
    // Utility functions
    long stepsFromMM(float mm) const;
    float mmFromSteps(long steps) const;
    long stepsFromDegrees(float degrees) const;
    float degreesFromSteps(long steps) const;
    
    // Thread-safe position updates
    bool lockAxis(TickType_t timeout = portMAX_DELAY);
    void unlockAxis();
    
    // Position synchronization
    void synchronizeWithEncoder(long encoderPosition);
    void synchronizeWithMPG();
    
    // Error handling
    void clearErrors();
    const char* getLastError() const;
    
private:
    // Error tracking
    volatile uint32_t errorFlags;
    char lastErrorMessage[64];
    
    // Step generation timing
    volatile unsigned long stepInterval;    // Current step interval (microseconds)
    volatile unsigned long nextStepTime;    // Next step time (microseconds)
    
    // Motion planning
    volatile long accelerationDistance;    // Distance for acceleration phase
    volatile long decelerationDistance;    // Distance for deceleration phase
    volatile long constantSpeedDistance;   // Distance at constant speed
    
    // Performance counters
    volatile unsigned long totalSteps;     // Total steps executed
    volatile unsigned long stepsPerSecond; // Current step rate
};

// Utility functions for axis management
inline bool isValidAxisName(char axis) {
    return (axis == 'X' || axis == 'Y' || axis == 'Z');
}

inline float convertToDisplayUnits(long steps, float stepsPerUnit, bool metric) {
    float value = steps / stepsPerUnit;
    if (!metric) {
        value /= 25.4f; // Convert mm to inches
    }
    return value;
}

inline long convertFromDisplayUnits(float value, float stepsPerUnit, bool metric) {
    if (!metric) {
        value *= 25.4f; // Convert inches to mm
    }
    return (long)(value * stepsPerUnit);
}

// Axis error flags
#define AXIS_ERROR_NONE             0x00000000
#define AXIS_ERROR_POSITION_LIMIT   0x00000001
#define AXIS_ERROR_SPEED_LIMIT      0x00000002
#define AXIS_ERROR_ACCELERATION     0x00000004
#define AXIS_ERROR_COMMUNICATION    0x00000008
#define AXIS_ERROR_ENCODER          0x00000010
#define AXIS_ERROR_MOTOR_STALL      0x00000020
#define AXIS_ERROR_HARDWARE         0x00000040
#define AXIS_ERROR_CONFIGURATION    0x00000080
#define AXIS_ERROR_EMERGENCY_STOP   0x00000100
#define AXIS_ERROR_LIMIT_SWITCH     0x00000200