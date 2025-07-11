/*
 * spindle_encoder.h
 * Spindle Encoder Class for nanoELS H5
 * 
 * Handles spindle encoder reading, position tracking, and RPM calculation
 * with enhanced velocity-based control and safety monitoring.
 */

#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config/hardware_config.h"
#include "config/safety_config.h"

// Encoder configuration structure
struct EncoderConfig {
    int encoderA;                    // Encoder A pin
    int encoderB;                    // Encoder B pin
    int encoderPPR;                  // Pulses per revolution
    int encoderBacklash;             // Backlash compensation pulses
    int encoderFilter;               // Filter for noise reduction
    pcnt_unit_t pcntUnit;           // Hardware pulse counter unit
    int pcntLimit;                   // Pulse counter limit
    int pcntClear;                   // Pulse counter clear threshold
};

// Spindle state structure
struct SpindleState {
    volatile long position;          // Current spindle position
    volatile long positionAvg;       // Average position (backlash compensated)
    volatile long positionGlobal;    // Global position (unaffected by zeroing)
    volatile unsigned long lastPulseTime; // Last pulse time (microseconds)
    volatile unsigned long pulseTimeDiff; // Time between pulses
    volatile unsigned long rpmBulkTime;   // Time for RPM calculation
    volatile int pulseCount;         // Current pulse counter value
    volatile int lastPulseCount;     // Previous pulse counter value
    volatile int direction;          // Spindle direction (1/-1)
    volatile float rpm;              // Current RPM
    volatile bool isActive;          // Whether spindle is rotating
    volatile bool isHealthy;         // Encoder health status
};

class SpindleEncoder {
private:
    // Configuration
    EncoderConfig config;
    SpindleState state;
    
    // Thread safety
    SemaphoreHandle_t encoderMutex;
    
    // RPM calculation
    volatile unsigned long rpmBulkTimeDiff;
    volatile unsigned long rpmTimeAtIndex0;
    volatile int rpmTimeIndex;
    volatile unsigned long rpmBulk;
    
    // Safety monitoring
    volatile unsigned long lastEncoderSignal;
    volatile uint32_t faultCount;
    volatile bool emergencyStopTriggered;
    
    // Private methods
    void initializeHardwareCounter();
    void configurePulseCounter();
    void processEncoderPulse();
    void calculateRPM();
    void updateDirection();
    bool validateEncoderHealth();
    void handleEncoderFault();
    
    // Hardware control
    void resetPulseCounter();
    void clearPulseCounter();
    int readPulseCounter();
    
public:
    SpindleEncoder();
    SpindleEncoder(const EncoderConfig& configuration);
    ~SpindleEncoder();
    
    // Initialization and configuration
    bool initializeEncoder();
    bool configureEncoder(const EncoderConfig& newConfig);
    EncoderConfig getConfiguration() const;
    bool validateConfiguration();
    
    // Encoder reading and processing
    void processEncoderSignal();
    void updateEncoderState();
    void resetEncoderPosition();
    void setEncoderPosition(long position);
    
    // Position and status queries
    long getCurrentPosition() const;
    long getAveragePosition() const;
    long getGlobalPosition() const;
    float getRPM() const;
    bool isSpindleActive() const;
    bool isEncoderHealthy() const;
    
    // Direction and movement
    int getDirection() const;
    bool isClockwise() const;
    bool isCounterClockwise() const;
    
    // RPM and velocity calculations
    float getVelocity() const;
    float getAngularVelocity() const;
    unsigned long getLastPulseTime() const;
    unsigned long getPulseTimeDiff() const;
    
    // Safety and monitoring
    void checkEncoderHealth();
    void reportEncoderFault();
    uint32_t getFaultCount() const;
    bool hasEmergencyStop() const;
    void clearEmergencyStop();
    
    // Calibration and diagnostics
    bool performSelfTest();
    bool calibrateEncoder();
    float getPositionError() const;
    unsigned long getUptimeSeconds() const;
    
    // Thread-safe operations
    bool lockEncoder(TickType_t timeout = portMAX_DELAY);
    void unlockEncoder();
    
    // Utility functions
    long pulsesFromDegrees(float degrees) const;
    float degreesFromPulses(long pulses) const;
    long pulsesFromRevolutions(float revolutions) const;
    float revolutionsFromPulses(long pulses) const;
    
    // Enhanced MPG velocity control
    float calculateVelocityFromPulses(int pulseDelta, unsigned long timeDelta);
    float getVelocityAverage(int samples) const;
    bool isVelocityStable() const;
    
private:
    // Velocity tracking for enhanced MPG control
    struct VelocityTracker {
        unsigned long lastPulseTimes[10];
        int pulseCounts[10];
        int sampleIndex;
        float currentVelocity;
        float averageVelocity;
        bool isActive;
    };
    
    VelocityTracker velocityTracker;
    
    // Velocity calculation methods
    void updateVelocityTracker(int pulseDelta);
    float calculateAverageVelocity();
    void resetVelocityTracker();
};

// Global spindle encoder instance
extern SpindleEncoder* g_spindleEncoder;

// Utility functions for encoder management
inline bool isValidEncoderPin(int pin) {
    return pin >= 0 && pin <= 48;
}

inline bool isValidPPR(int ppr) {
    return ppr > 0 && ppr <= 10000;
}

inline float convertPulsesToDegrees(long pulses, int ppr) {
    return (pulses * 360.0f) / ppr;
}

inline long convertDegreesToPulses(float degrees, int ppr) {
    return (degrees * ppr) / 360.0f;
}

// Encoder interrupt service routines
void IRAM_ATTR encoderISR();
void IRAM_ATTR encoderAISR();
void IRAM_ATTR encoderBISR();