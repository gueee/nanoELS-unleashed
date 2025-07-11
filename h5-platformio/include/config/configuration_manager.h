/*
 * configuration_manager.h
 * Configuration Manager Class for nanoELS H5
 * 
 * Handles system configuration, preferences storage, and settings management
 * with persistent storage and runtime configuration updates.
 */

#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "config/hardware_config.h"
#include "config/safety_config.h"
#include "core/axis_controller.h"

// Configuration version management
#define CONFIG_VERSION 1
#define CONFIG_NAMESPACE "h5"
#define GCODE_NAMESPACE "gc"

// System configuration structure
struct SystemConfig {
    // Hardware configuration
    int encoderPPR;                  // Encoder pulses per revolution
    int encoderBacklash;             // Encoder backlash compensation
    bool wifiEnabled;                // WiFi enabled
    bool keyboardEnabled;            // PS2 keyboard enabled
    bool displayEnabled;             // Display enabled
    
    // Motion configuration
    float defaultFeedRate;           // Default feed rate
    float maxFeedRate;               // Maximum feed rate
    float minFeedRate;               // Minimum feed rate
    int defaultRPM;                  // Default spindle RPM
    int maxRPM;                      // Maximum spindle RPM
    int minRPM;                      // Minimum spindle RPM
    
    // Safety configuration
    bool emergencyStopEnabled;       // Emergency stop enabled
    bool softLimitsEnabled;          // Soft limits enabled
    bool hardLimitsEnabled;          // Hard limits enabled
    int safetyTimeout;               // Safety timeout (ms)
    int watchdogTimeout;             // Watchdog timeout (ms)
    
    // Display configuration
    int displayBrightness;           // Display brightness (0-100)
    int displayTimeout;              // Display timeout (ms)
    bool displayAutoDim;             // Auto dim display
    int displayUpdateInterval;       // Display update interval (ms)
    
    // Communication configuration
    int webServerPort;               // Web server port
    int webSocketPort;               // WebSocket port
    int maxConnections;              // Maximum connections
    int communicationTimeout;        // Communication timeout (ms)
    
    // G-code configuration
    bool gcodeEnabled;               // G-code enabled
    int gcodeBufferSize;             // G-code buffer size
    bool gcodeComments;              // G-code comments enabled
    int gcodeTimeout;                // G-code timeout (ms)
};

// Axis configuration structure (extends AxisConfig from axis_controller.h)
struct ExtendedAxisConfig {
    AxisConfig axisConfig;           // Basic axis configuration
    
    // Extended settings
    bool enabled;                    // Axis enabled
    bool homed;                      // Axis homed
    float homePosition;              // Home position
    float maxTravel;                 // Maximum travel
    float softLimitMin;              // Soft limit minimum
    float softLimitMax;              // Soft limit maximum
    bool hardLimitEnabled;           // Hard limit enabled
    int hardLimitMinPin;             // Hard limit minimum pin
    int hardLimitMaxPin;             // Hard limit maximum pin
    bool invertHardLimits;           // Invert hard limit signals
    
    // Calibration settings
    float stepsPerUnit;              // Steps per unit
    float backlashCompensation;      // Backlash compensation
    float leadScrewPitch;            // Lead screw pitch
    bool calibrated;                 // Axis calibrated
};

// User preferences structure
struct UserPreferences {
    // Measurement system
    int measurementSystem;           // 0=metric, 1=imperial, 2=TPI
    bool showAngle;                  // Show spindle angle
    bool showRPM;                    // Show spindle RPM
    bool showPosition;               // Show position
    bool showLimits;                 // Show limits
    
    // Display preferences
    int displayMode;                 // Display mode
    int displayUnits;                // Display units
    bool displayInverted;            // Display inverted
    int displayRefreshRate;          // Display refresh rate
    
    // Operation preferences
    int defaultMode;                 // Default operation mode
    float defaultPitch;              // Default pitch
    int defaultStarts;               // Default starts
    int defaultPasses;               // Default passes
    float defaultConeRatio;          // Default cone ratio
    
    // Safety preferences
    bool confirmEmergencyStop;       // Confirm emergency stop
    bool confirmModeChange;          // Confirm mode change
    bool confirmLimitChange;         // Confirm limit change
    int safetyConfirmationTimeout;   // Safety confirmation timeout
};

class ConfigurationManager {
private:
    // Configuration storage
    Preferences systemPrefs;
    Preferences gcodePrefs;
    
    // Configuration data
    SystemConfig systemConfig;
    ExtendedAxisConfig axisConfigs[3]; // X, Y, Z axes
    UserPreferences userPrefs;
    
    // Thread safety
    SemaphoreHandle_t configMutex;
    
    // Configuration tracking
    volatile bool configChanged;
    volatile unsigned long lastSaveTime;
    volatile unsigned long saveInterval;
    
    // Private methods
    void initializePreferences();
    void loadSystemConfiguration();
    void loadAxisConfigurations();
    void loadUserPreferences();
    void saveSystemConfiguration();
    void saveAxisConfigurations();
    void saveUserPreferences();
    void validateConfiguration();
    void migrateConfiguration();
    
    // Configuration validation
    bool validateSystemConfig(const SystemConfig& config);
    bool validateAxisConfig(const ExtendedAxisConfig& config);
    bool validateUserPrefs(const UserPreferences& prefs);
    
    // Default configurations
    void setDefaultSystemConfig();
    void setDefaultAxisConfigs();
    void setDefaultUserPrefs();
    
public:
    ConfigurationManager();
    ~ConfigurationManager();
    
    // Initialization and management
    bool initializeConfiguration();
    bool loadConfiguration();
    bool saveConfiguration();
    bool resetConfiguration();
    bool validateConfiguration();
    
    // System configuration
    bool setSystemConfig(const SystemConfig& config);
    SystemConfig getSystemConfig() const;
    bool updateSystemConfig(const SystemConfig& config);
    
    // Axis configuration
    bool setAxisConfig(char axis, const ExtendedAxisConfig& config);
    ExtendedAxisConfig getAxisConfig(char axis) const;
    bool updateAxisConfig(char axis, const ExtendedAxisConfig& config);
    bool isAxisEnabled(char axis) const;
    bool isAxisHomed(char axis) const;
    
    // User preferences
    bool setUserPreferences(const UserPreferences& prefs);
    UserPreferences getUserPreferences() const;
    bool updateUserPreferences(const UserPreferences& prefs);
    
    // Configuration queries
    bool isConfigurationValid() const;
    bool hasConfigurationChanged() const;
    unsigned long getLastSaveTime() const;
    int getConfigurationVersion() const;
    
    // Thread-safe operations
    bool lockConfiguration(TickType_t timeout = portMAX_DELAY);
    void unlockConfiguration();
    
    // Configuration utilities
    bool exportConfiguration(char* buffer, size_t bufferSize);
    bool importConfiguration(const char* buffer, size_t bufferSize);
    bool backupConfiguration(const char* filename);
    bool restoreConfiguration(const char* filename);
    
    // Runtime configuration
    bool setRuntimeConfig(const char* key, const char* value);
    bool getRuntimeConfig(const char* key, char* value, size_t valueSize);
    bool deleteRuntimeConfig(const char* key);
    
    // G-code file management
    bool saveGCodeFile(const char* filename, const char* content);
    bool loadGCodeFile(const char* filename, char* buffer, size_t bufferSize);
    bool deleteGCodeFile(const char* filename);
    int getGCodeFileCount() const;
    void listGCodeFiles(char* buffer, size_t bufferSize);
    
    // Enhanced configuration features
    void setMeasurementSystem(int system);
    void setDisplayMode(int mode);
    void setDefaultMode(int mode);
    void setSafetySettings(bool emergencyStop, bool softLimits, bool hardLimits);
    
    // Configuration validation and diagnostics
    bool performConfigurationTest();
    bool testAxisConfiguration(char axis);
    bool testSystemConfiguration();
    bool testUserPreferences();
    
    // Migration and version management
    bool migrateFromVersion(int oldVersion);
    bool upgradeConfiguration();
    bool downgradeConfiguration();
    
    // Configuration statistics
    unsigned long getConfigurationSize() const;
    unsigned long getGCodeStorageSize() const;
    unsigned long getFreeStorageSize() const;
    float getStorageUsagePercent() const;
    
private:
    // Configuration storage management
    bool initializeStorage();
    bool validateStorage();
    void cleanupStorage();
    
    // Configuration versioning
    int getStoredVersion() const;
    void setStoredVersion(int version);
    bool needsMigration() const;
    
    // Error handling
    void handleConfigurationError(const char* error);
    void resetErrorState();
    bool hasConfigurationError() const;
    
    // Configuration change tracking
    void markConfigurationChanged();
    void clearConfigurationChanged();
    bool shouldAutoSave() const;
};

// Global configuration manager instance
extern ConfigurationManager* g_configManager;

// Configuration utility functions
inline bool isValidConfigVersion(int version) {
    return version >= 1 && version <= 10;
}

inline bool isValidMeasurementSystem(int system) {
    return system >= 0 && system <= 2;
}

inline bool isValidDisplayMode(int mode) {
    return mode >= 0 && mode <= 5;
}

// Configuration command macros
#define CONFIG_SAVE() do { \
    if (g_configManager) { \
        g_configManager->saveConfiguration(); \
    } \
} while(0)

#define CONFIG_LOAD() do { \
    if (g_configManager) { \
        g_configManager->loadConfiguration(); \
    } \
} while(0)

#define CONFIG_GET_AXIS(axis) (g_configManager ? g_configManager->getAxisConfig(axis) : ExtendedAxisConfig())

// Configuration validation macros
#define CONFIG_CHECK() do { \
    if (!g_configManager || !g_configManager->isConfigurationValid()) { \
        return false; \
    } \
} while(0)

#define CONFIG_CHECK_RETURN(value) do { \
    if (!g_configManager || !g_configManager->isConfigurationValid()) { \
        return value; \
    } \
} while(0)