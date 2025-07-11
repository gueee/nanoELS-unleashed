/*
 * motion_planner.h
 * Motion Planner Class for nanoELS H5
 * 
 * Handles multi-axis motion planning, G-code interpretation, and coordinated
 * movement with safety monitoring and real-time path planning.
 */

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "config/hardware_config.h"
#include "config/safety_config.h"
#include "core/axis_controller.h"
#include "core/spindle_encoder.h"
#include "core/safety_system.h"

// Motion planning configuration
struct MotionConfig {
    float interpolationPrecision;     // Linear interpolation precision
    long gcodeWaitEpsilon;           // G-code wait epsilon in steps
    bool spindlePausesGcode;         // Whether spindle stops pause G-code
    int gcodeMinRPM;                 // Minimum RPM for G-code execution
    long gcodeFeedDefault;           // Default feed rate in du/sec
    float gcodeFeedMin;              // Minimum feed rate in du/sec
    long gcodeFeedMax;               // Maximum feed rate in du/sec
};

// G-code command structure
struct GCodeCommand {
    char command;                    // G, M, T, etc.
    int number;                      // Command number (G1, M3, etc.)
    float x, y, z, f, s;            // Coordinates and feed/speed
    bool hasX, hasY, hasZ, hasF, hasS; // Whether parameters are present
    char* comment;                   // Comment string
    int lineNumber;                  // Line number for error reporting
};

// Motion state enumeration
enum MotionState {
    MOTION_IDLE = 0,
    MOTION_PLANNING = 1,
    MOTION_EXECUTING = 2,
    MOTION_PAUSED = 3,
    MOTION_ERROR = 4,
    MOTION_COMPLETE = 5
};

// G-code execution state
struct GCodeState {
    bool absoluteMode;               // G90/G91 mode
    bool inchMode;                   // G20/G21 mode
    float currentX, currentY, currentZ; // Current position
    float currentF, currentS;        // Current feed and speed
    int currentTool;                 // Current tool number
    bool coolantOn;                  // Coolant state
    bool spindleOn;                  // Spindle state
    int spindleDirection;            // Spindle direction (1/-1)
    MotionState motionState;         // Current motion state
    int errorCode;                   // Error code if any
    char errorMessage[64];           // Error message
};

class MotionPlanner {
private:
    // Configuration
    MotionConfig config;
    GCodeState gcodeState;
    
    // Axis controllers
    AxisController* axisZ;
    AxisController* axisX;
    AxisController* axisY;
    
    // System components
    SpindleEncoder* spindleEncoder;
    SafetySystem* safetySystem;
    
    // Thread safety
    SemaphoreHandle_t plannerMutex;
    QueueHandle_t gcodeQueue;
    
    // Motion planning
    volatile MotionState currentState;
    volatile bool isExecuting;
    volatile bool isPaused;
    volatile unsigned long lastPlanTime;
    
    // G-code processing
    char* gcodeBuffer;
    size_t gcodeBufferSize;
    size_t gcodeBufferPos;
    int gcodeLineNumber;
    bool gcodeEndOfFile;
    
    // Path planning
    struct PathSegment {
        float startX, startY, startZ;
        float endX, endY, endZ;
        float feedRate;
        bool isRapid;
        bool isLinear;
        bool isArc;
    };
    
    PathSegment currentSegment;
    bool hasCurrentSegment;
    
    // Private methods
    void initializeGCodeState();
    void parseGCodeLine(const char* line, GCodeCommand* cmd);
    bool validateGCodeCommand(const GCodeCommand* cmd);
    void executeGCodeCommand(const GCodeCommand* cmd);
    void planLinearMove(float x, float y, float z, float f);
    void planRapidMove(float x, float y, float z);
    void planArcMove(float x, float y, float z, float i, float j, float k, float f);
    bool interpolatePath();
    void updateMotionState();
    void handleMotionError(int errorCode, const char* message);
    
    // Safety and monitoring
    bool validateMotionCommand(const GCodeCommand* cmd);
    bool checkSafetyLimits(float x, float y, float z);
    void monitorMotionExecution();
    
public:
    MotionPlanner(AxisController* z, AxisController* x, AxisController* y, 
                  SpindleEncoder* encoder, SafetySystem* safety);
    ~MotionPlanner();
    
    // Initialization and configuration
    bool initializePlanner();
    bool configurePlanner(const MotionConfig& newConfig);
    MotionConfig getConfiguration() const;
    bool validateConfiguration();
    
    // Motion planning
    void planMotion();
    bool planLinearPath(float startX, float startY, float startZ,
                       float endX, float endY, float endZ, float feedRate);
    bool planArcPath(float centerX, float centerY, float centerZ,
                    float endX, float endY, float endZ, float feedRate, bool clockwise);
    void stopMotion(bool immediate = false);
    void pauseMotion();
    void resumeMotion();
    
    // G-code processing
    bool loadGCode(const char* filename);
    bool loadGCodeFromBuffer(const char* buffer, size_t size);
    void processGCode();
    bool executeGCodeLine(const char* line);
    void resetGCodeState();
    
    // Motion state queries
    MotionState getCurrentState() const;
    bool isExecuting() const;
    bool isPaused() const;
    bool hasError() const;
    int getErrorCode() const;
    const char* getErrorMessage() const;
    
    // Position and status
    void getCurrentPosition(float& x, float& y, float& z) const;
    void getTargetPosition(float& x, float& y, float& z) const;
    float getCurrentFeedRate() const;
    float getCurrentSpeed() const;
    
    // G-code state management
    GCodeState getGCodeState() const;
    void setGCodeState(const GCodeState& state);
    bool isAbsoluteMode() const;
    bool isInchMode() const;
    
    // Safety and monitoring
    bool validateMotion(float x, float y, float z, float f);
    void checkMotionSafety();
    bool isMotionSafe() const;
    
    // File management
    bool saveGCode(const char* filename, const char* content);
    bool loadGCodeFile(const char* filename);
    bool deleteGCodeFile(const char* filename);
    int getGCodeFileCount() const;
    void listGCodeFiles(char* buffer, size_t bufferSize) const;
    
    // Thread-safe operations
    bool lockPlanner(TickType_t timeout = portMAX_DELAY);
    void unlockPlanner();
    
    // Utility functions
    float convertToMetric(float value) const;
    float convertToInch(float value) const;
    float convertFeedRate(float feedRate) const;
    bool isCoordinateValid(float value) const;
    
    // Enhanced motion features
    void setSpindleSpeed(float rpm);
    void setSpindleDirection(int direction);
    void setCoolant(bool on);
    void setTool(int toolNumber);
    
    // Diagnostics and monitoring
    unsigned long getExecutionTime() const;
    unsigned long getPlanningTime() const;
    int getProcessedLines() const;
    bool performSelfTest();
    
private:
    // Execution timing
    volatile unsigned long executionStartTime;
    volatile unsigned long planningStartTime;
    volatile int processedLines;
    
    // Error handling
    volatile int lastErrorCode;
    volatile char lastErrorMessage[64];
    
    // File system integration
    bool initializeFileSystem();
    size_t getFileSize(const char* filename) const;
    bool fileExists(const char* filename) const;
};

// Global motion planner instance
extern MotionPlanner* g_motionPlanner;

// Utility functions for motion planning
inline bool isValidCoordinate(float value) {
    return !isnan(value) && isfinite(value);
}

inline bool isValidFeedRate(float feedRate) {
    return feedRate > 0 && feedRate < 1000000;
}

inline float convertMMToInch(float mm) {
    return mm / 25.4f;
}

inline float convertInchToMM(float inch) {
    return inch * 25.4f;
}

// Motion planning utility macros
#define MOTION_CHECK() do { \
    if (!g_motionPlanner || !g_motionPlanner->isMotionSafe()) { \
        return false; \
    } \
} while(0)

#define MOTION_CHECK_RETURN(value) do { \
    if (!g_motionPlanner || !g_motionPlanner->isMotionSafe()) { \
        return value; \
    } \
} while(0)