/*
 * safety_system.h
 * Safety System Class for nanoELS H5
 * 
 * Implements comprehensive safety monitoring and emergency stop functionality
 * for the lathe controller. This is the highest priority safety-critical component.
 */

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include "config/hardware_config.h"
#include "config/safety_config.h"

class SafetySystem {
private:
    // Safety state variables
    volatile SafetySystemState currentState;
    volatile EmergencyStopType activeEmergencyStop;
    volatile EmergencyStopPriority currentPriority;
    
    // Hardware monitoring
    volatile bool hardwareEmergencyStopPressed;
    volatile bool limitSwitchTriggered;
    volatile bool watchdogActive;
    volatile unsigned long lastWatchdogFeed;
    volatile unsigned long lastSafetyCheck;
    
    // Fault tracking
    volatile uint32_t faultCount;
    volatile uint32_t encoderFaultCount;
    volatile unsigned long lastEncoderSignal;
    
    // Thread safety
    SemaphoreHandle_t safetyMutex;
    TaskHandle_t watchdogTask;
    
    // Hardware monitoring methods
    bool checkHardwareEmergencyStop();
    bool checkHardwareLimitSwitches();
    bool checkEncoderHealth();
    bool checkPowerSupply();
    bool checkTemperature();
    
    // Internal safety validation
    bool validateSystemIntegrity();
    void updateSafetyState(SafetySystemState newState);
    void logSafetyEvent(EmergencyStopType type, const char* message);
    
    // Watchdog management
    static void watchdogTaskFunction(void* parameter);
    void initializeWatchdog();
    void configureHardwareWatchdog();
    
public:
    SafetySystem();
    ~SafetySystem();
    
    // Initialization and configuration
    bool initializeHardwareEmergencyStop();
    bool initializeLimitSwitches();
    bool initializeSafetyMonitoring();
    bool validateConfiguration();
    
    // Emergency stop management
    void triggerEmergencyStop(EmergencyStopType type);
    void clearEmergencyStop();
    bool resetEmergencyStop();
    bool acknowledgeEmergencyStop();
    
    // Safety state queries
    bool isSystemSafe() const;
    bool isEmergencyStopActive() const;
    SafetySystemState getCurrentState() const;
    EmergencyStopType getActiveEmergencyStop() const;
    const char* getSafetyStatusMessage() const;
    
    // Safety monitoring
    void checkEmergencyStop();
    void checkHardwareLimits();
    void checkSystemHealth();
    void performSafetyCheck();
    
    // Watchdog management
    void feedWatchdog();
    void enableWatchdog();
    void disableWatchdog();
    bool isWatchdogHealthy() const;
    
    // Position and motion safety
    bool validatePosition(char axis, long position);
    bool validateSpeed(char axis, long speed);
    bool validateAcceleration(char axis, long acceleration);
    bool validateMotionCommand(char axis, long position, long speed);
    
    // Hardware fault detection
    void reportEncoderFault();
    void reportMotorStall(char axis);
    void reportCommunicationFault();
    void reportHardwareFault(const char* description);
    
    // Safety statistics and diagnostics
    uint32_t getFaultCount() const;
    uint32_t getEncoderFaultCount() const;
    unsigned long getLastSafetyCheckTime() const;
    unsigned long getUptimeSeconds() const;
    
    // Maintenance and testing
    bool performSelfTest();
    bool testEmergencyStop();
    bool testLimitSwitches();
    bool testWatchdog();
    
    // Safety override (use with extreme caution)
    bool requestSafetyOverride(const char* reason, uint32_t duration_ms);
    void cancelSafetyOverride();
    bool isSafetyOverrideActive() const;
};

// Global safety system instance (singleton pattern for safety-critical system)
extern SafetySystem* g_safetySystem;

// Safety utility functions
inline bool isMotionSafe() {
    return g_safetySystem && g_safetySystem->isSystemSafe();
}

inline void emergencyStopAll() {
    if (g_safetySystem) {
        g_safetySystem->triggerEmergencyStop(ESTOP_USER_REQUESTED);
    }
}

inline bool validateSafeMotion(char axis, long position, long speed) {
    return g_safetySystem && 
           g_safetySystem->validatePosition(axis, position) &&
           g_safetySystem->validateSpeed(axis, speed);
}

// Safety macros for common checks
#define SAFETY_CHECK() do { \
    if (!isMotionSafe()) { \
        return false; \
    } \
} while(0)

#define SAFETY_CHECK_RETURN(value) do { \
    if (!isMotionSafe()) { \
        return value; \
    } \
} while(0)

#define SAFETY_CHECK_VOID() do { \
    if (!isMotionSafe()) { \
        return; \
    } \
} while(0)

// Emergency stop interrupt service routine
void IRAM_ATTR emergencyStopISR();

// Hardware limit switch interrupt service routines
void IRAM_ATTR limitSwitchZMinISR();
void IRAM_ATTR limitSwitchZMaxISR();
void IRAM_ATTR limitSwitchXMinISR();
void IRAM_ATTR limitSwitchXMaxISR();
void IRAM_ATTR limitSwitchYMinISR();
void IRAM_ATTR limitSwitchYMaxISR();