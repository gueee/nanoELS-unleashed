/*
 * display_manager.h
 * Display Manager Class for nanoELS H5
 * 
 * Handles Nextion touch screen interface, display updates, and user interface
 * management with real-time status display and touch input processing.
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "config/hardware_config.h"
#include "core/axis_controller.h"
#include "core/spindle_encoder.h"

// Display configuration
struct DisplayConfig {
    int serialTxPin;                 // Serial TX pin for Nextion
    int serialRxPin;                 // Serial RX pin for Nextion
    long baudRate;                   // Serial baud rate
    int updateInterval;              // Display update interval (ms)
    bool enableTouch;                // Enable touch input
    bool enableBacklight;            // Enable backlight control
    int backlightLevel;              // Backlight brightness (0-100)
};

// Display state structure
struct DisplayState {
    bool isConnected;                // Whether display is connected
    bool isInitialized;              // Whether display is initialized
    bool isUpdating;                 // Whether display is being updated
    unsigned long lastUpdateTime;    // Last update time
    unsigned long connectionTime;    // Connection time
    int errorCount;                  // Communication error count
    char lastError[64];              // Last error message
};

// Display data structure
struct DisplayData {
    // System status
    bool systemOn;                   // System on/off state
    bool emergencyStop;              // Emergency stop state
    int currentMode;                 // Current operation mode
    int currentMeasure;              // Current measurement system
    
    // Axis positions
    float posX, posY, posZ;          // Current positions
    float targetX, targetY, targetZ; // Target positions
    bool axisEnabled[3];             // Axis enable states
    
    // Spindle information
    float spindleRPM;                // Current spindle RPM
    float spindleAngle;              // Current spindle angle
    bool spindleActive;              // Spindle active state
    int spindleDirection;            // Spindle direction
    
    // Operation parameters
    float pitch;                     // Current pitch
    int starts;                      // Number of starts
    int passes;                      // Number of passes
    float coneRatio;                 // Cone ratio
    
    // Limits and stops
    float leftStopX, rightStopX;     // X axis limits
    float leftStopZ, rightStopZ;     // Z axis limits
    bool limitsEnabled;              // Soft limits enabled
    
    // G-code information
    bool gcodeActive;                // G-code execution active
    int gcodeLine;                   // Current G-code line
    int gcodeTotalLines;             // Total G-code lines
    float gcodeProgress;             // G-code progress (0-100)
};

class DisplayManager {
private:
    // Configuration
    DisplayConfig config;
    DisplayState state;
    DisplayData data;
    
    // Hardware serial
    HardwareSerial* displaySerial;
    
    // Thread safety
    SemaphoreHandle_t displayMutex;
    QueueHandle_t touchQueue;
    
    // Display update tracking
    volatile bool updateRequired;
    volatile bool forceUpdate;
    volatile unsigned long lastHash;
    volatile unsigned long updateCounter;
    
    // Touch input processing - forward declaration
    // (TouchEvent moved to public section)
    
    // Private methods
    void initializeSerial();
    bool initializeDisplay();
    void processDisplayResponse();
    void updateDisplayData();
    void sendDisplayCommand(const char* command);
    void handleTouchEvent(const TouchEvent& event);
    void calculateDisplayHash();
    bool validateDisplayConnection();
    
    // Display commands
    void updateSystemStatus();
    void updateAxisPositions();
    void updateSpindleInfo();
    void updateOperationParams();
    void updateLimitsAndStops();
    void updateGCodeInfo();
    
    // Additional methods for compatibility
    void initializeState();
    void setText(const char* id, const char* text);
    bool testCommunication();
    void handleDisplayError(const char* error);
    
    // Touch input handling
    void processTouchInput();
    void handleButtonPress(int buttonId);
    void handleNumericInput(int digit);
    void handleModeChange(int mode);
    
public:
    DisplayManager();
    DisplayManager(const DisplayConfig& configuration);
    ~DisplayManager();
    
    // Initialization and configuration
    bool initializeDisplay();
    bool configureDisplay(const DisplayConfig& newConfig);
    DisplayConfig getConfiguration() const;
    bool validateConfiguration();
    
    // Display control
    void updateDisplay();
    void forceDisplayUpdate();
    void clearDisplay();
    void setBacklight(int level);
    void setBrightness(int level);
    
    // Data updates
    void updateSystemStatus(bool on, bool estop, int mode, int measure);
    void updateAxisPositions(float x, float y, float z);
    void updateAxisTargets(float x, float y, float z);
    void updateAxisEnabled(bool x, bool y, bool z);
    void updateSpindleInfo(float rpm, float angle, bool active, int direction);
    void updateOperationParams(float pitch, int starts, int passes, float coneRatio);
    void updateLimitsAndStops(float leftX, float rightX, float leftZ, float rightZ, bool enabled);
    void updateGCodeInfo(bool active, int line, int total, float progress);
    
    // Touch input
    void processTouchInput();
    void handleTouchEvent(int x, int y, bool pressed);
    bool hasTouchEvent() const;
    TouchEvent getNextTouchEvent();
    
    // Display state queries
    bool isConnected() const;
    bool isInitialized() const;
    bool isUpdating() const;
    int getErrorCount() const;
    const char* getLastError() const;
    unsigned long getUptimeSeconds() const;
    
    // Communication
    bool sendCommand(const char* command);
    bool sendCommand(const char* command, const char* parameter);
    bool sendCommand(const char* command, int parameter);
    bool sendCommand(const char* command, float parameter);
    bool readResponse(char* buffer, size_t bufferSize, unsigned long timeout = 1000);
    
    // Touch event structure (moved here for visibility)
    struct TouchEvent {
        int x, y;                    // Touch coordinates
        bool pressed;                 // Press state
        unsigned long timestamp;      // Event timestamp
    };
    
    // Thread-safe operations
    bool lockDisplay(TickType_t timeout = portMAX_DELAY);
    void unlockDisplay();
    
    // Utility functions
    void formatPosition(char* buffer, size_t bufferSize, float position, int measure) const;
    void formatRPM(char* buffer, size_t bufferSize, float rpm) const;
    void formatAngle(char* buffer, size_t bufferSize, float angle) const;
    void formatPitch(char* buffer, size_t bufferSize, float pitch, int measure) const;
    
    // Enhanced display features
    void showSplashScreen();
    void showMainScreen();
    void showSettingsScreen();
    void showGCodeScreen();
    void showErrorScreen(const char* error);
    void showProgressBar(float progress);
    
    // Diagnostics and monitoring
    bool performSelfTest();
    bool testCommunication();
    bool testTouchInput();
    unsigned long getUpdateCount() const;
    float getUpdateRate() const;
    
    // Display data access
    DisplayData getDisplayData() const;
    void setDisplayData(const DisplayData& data);
    
private:
    // Communication buffers
    char commandBuffer[256];
    char responseBuffer[256];
    
    // Touch event queue
    static const int MAX_TOUCH_EVENTS = 10;
    TouchEvent touchEvents[MAX_TOUCH_EVENTS];
    int touchEventCount;
    int touchEventIndex;
    
    // Display hash calculation
    unsigned long calculateDataHash() const;
    bool hasDataChanged() const;
    
    // Error handling
    void handleDisplayError(const char* error);
    void resetErrorCount();
    void incrementErrorCount();
};

// Global display manager instance
extern DisplayManager* g_displayManager;

// Display utility functions
inline bool isValidDisplayPin(int pin) {
    return pin >= 0 && pin <= 48;
}

inline bool isValidBaudRate(long baudRate) {
    return baudRate >= 9600 && baudRate <= 115200;
}

inline bool isValidBacklightLevel(int level) {
    return level >= 0 && level <= 100;
}

// Display command macros
#define DISPLAY_COMMAND(cmd) do { \
    if (g_displayManager) { \
        g_displayManager->sendCommand(cmd); \
    } \
} while(0)

#define DISPLAY_COMMAND_PARAM(cmd, param) do { \
    if (g_displayManager) { \
        g_displayManager->sendCommand(cmd, param); \
    } \
} while(0)

// Nextion display commands
#define NEXTION_UPDATE_VAR(var, value) DISPLAY_COMMAND_PARAM(var, value)
#define NEXTION_SET_PAGE(page) DISPLAY_COMMAND_PARAM("page", page)
#define NEXTION_REFRESH() DISPLAY_COMMAND("ref")
#define NEXTION_CLEAR() DISPLAY_COMMAND("cls")