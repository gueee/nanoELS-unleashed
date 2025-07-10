/*
 * nanoELS H5 - Electronic Lead Screw Controller
 * ESP32-S3 based lathe controller with safety-first design
 * 
 * Original work by Maxim Kachurovskiy: https://github.com/kachurovskiy/nanoels
 * Enhanced version with class-based architecture and PlatformIO support
 * 
 * Hardware: ESP32-S3, SN74HCT245N buffers, Nextion display, PS2 keyboard
 * Platform: PlatformIO with Arduino Framework
 * 
 * SAFETY WARNING: This controls dangerous machinery. Always implement proper
 * emergency stops and safety measures before operating.
 */

#include <Arduino.h>
#include "config/hardware_config.h"
#include "config/safety_config.h"
#include "core/safety_system.h"
#include "core/axis_controller.h"
#include "core/spindle_encoder.h"
#include "core/motion_planner.h"
#include "ui/display_manager.h"
#include "comm/communication_manager.h"
#include "config/configuration_manager.h"

// Global system components
SafetySystem* safetySystem = nullptr;
AxisController* axisZ = nullptr;
AxisController* axisX = nullptr;
AxisController* axisY = nullptr;
SpindleEncoder* spindleEncoder = nullptr;
MotionPlanner* motionPlanner = nullptr;
DisplayManager* displayManager = nullptr;
CommunicationManager* communicationManager = nullptr;
ConfigurationManager* configManager = nullptr;

// FreeRTOS task handles
TaskHandle_t taskMotionControl = NULL;
TaskHandle_t taskSafetyMonitor = NULL;
TaskHandle_t taskDisplay = NULL;
TaskHandle_t taskCommunication = NULL;

// Motion control task - runs on Core 1 for real-time performance
void motionControlTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms cycle time
    
    while (true) {
        // Check safety system first
        if (!safetySystem->isSystemSafe()) {
            safetySystem->triggerEmergencyStop(ESTOP_SAFETY_VIOLATION);
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }
        
        // Feed watchdog
        safetySystem->feedWatchdog();
        
        // Process spindle encoder
        spindleEncoder->processEncoderSignal();
        
        // Execute motion planning
        motionPlanner->planMotion();
        
        // Update all axes
        if (axisZ && axisZ->isEnabled()) {
            axisZ->updateMotionProfile();
        }
        if (axisX && axisX->isEnabled()) {
            axisX->updateMotionProfile();
        }
        if (axisY && axisY->isEnabled()) {
            axisY->updateMotionProfile();
        }
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Safety monitoring task - runs on Core 0
void safetyMonitorTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms cycle time
    
    while (true) {
        // Check emergency stop
        safetySystem->checkEmergencyStop();
        
        // Check hardware limits
        safetySystem->checkHardwareLimits();
        
        // Monitor axis positions
        if (axisZ && !axisZ->isWithinLimits(axisZ->getCurrentPosition())) {
            safetySystem->triggerEmergencyStop(ESTOP_POS_LIMIT);
        }
        if (axisX && !axisX->isWithinLimits(axisX->getCurrentPosition())) {
            safetySystem->triggerEmergencyStop(ESTOP_POS_LIMIT);
        }
        if (axisY && !axisY->isWithinLimits(axisY->getCurrentPosition())) {
            safetySystem->triggerEmergencyStop(ESTOP_POS_LIMIT);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Display update task - runs on Core 0
void displayTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms cycle time (20Hz)
    
    while (true) {
        if (displayManager) {
            displayManager->updateDisplay();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Communication task - runs on Core 0
void communicationTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms cycle time
    
    while (true) {
        if (communicationManager) {
            communicationManager->processKeyboardInput();
            communicationManager->handleWebRequests();
            communicationManager->processWebSocketMessages();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("nanoELS H5 - Electronic Lead Screw Controller");
    Serial.println("Safety-first design with class-based architecture");
    Serial.println("Hardware: ESP32-S3, SN74HCT245N, Nextion, PS2");
    Serial.println("========================================");
    
    // Initialize configuration manager first
    configManager = new ConfigurationManager();
    if (!configManager->loadConfiguration()) {
        Serial.println("WARNING: Failed to load configuration, using defaults");
    }
    
    // Initialize safety system (highest priority)
    safetySystem = new SafetySystem();
    if (!safetySystem->initializeHardwareEmergencyStop()) {
        Serial.println("ERROR: Failed to initialize hardware emergency stop");
        while (true) {
            delay(1000);
            Serial.println("SAFETY ERROR: Cannot start without emergency stop");
        }
    }
    
    // Initialize spindle encoder
    spindleEncoder = new SpindleEncoder();
    if (!spindleEncoder->initializeEncoder()) {
        Serial.println("ERROR: Failed to initialize spindle encoder");
        safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
    }
    
    // Initialize axis controllers
    AxisConfig configZ = configManager->getAxisConfig('Z');
    axisZ = new AxisController('Z', configZ);
    if (!axisZ->initializeAxis()) {
        Serial.println("ERROR: Failed to initialize Z axis");
        safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
    }
    
    AxisConfig configX = configManager->getAxisConfig('X');
    axisX = new AxisController('X', configX);
    if (!axisX->initializeAxis()) {
        Serial.println("ERROR: Failed to initialize X axis");
        safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
    }
    
    // Initialize Y axis if enabled
    if (configManager->isYAxisEnabled()) {
        AxisConfig configY = configManager->getAxisConfig('Y');
        axisY = new AxisController('Y', configY);
        if (!axisY->initializeAxis()) {
            Serial.println("ERROR: Failed to initialize Y axis");
            safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
        }
    }
    
    // Initialize motion planner
    motionPlanner = new MotionPlanner(axisZ, axisX, axisY, spindleEncoder, safetySystem);
    
    // Initialize display manager
    displayManager = new DisplayManager();
    if (!displayManager->initializeDisplay()) {
        Serial.println("WARNING: Failed to initialize display");
    }
    
    // Initialize communication manager
    communicationManager = new CommunicationManager();
    if (!communicationManager->initializeCommunication()) {
        Serial.println("WARNING: Failed to initialize communication");
    }
    
    // Create FreeRTOS tasks
    Serial.println("Creating FreeRTOS tasks...");
    
    // Motion control task - Core 1, highest priority
    xTaskCreatePinnedToCore(
        motionControlTask,      // Task function
        "MotionControl",        // Name
        10000,                  // Stack size
        NULL,                   // Parameter
        configMAX_PRIORITIES - 1, // Priority (highest)
        &taskMotionControl,     // Task handle
        1                       // Core 1
    );
    
    // Safety monitoring task - Core 0, high priority
    xTaskCreatePinnedToCore(
        safetyMonitorTask,      // Task function
        "SafetyMonitor",        // Name
        4000,                   // Stack size
        NULL,                   // Parameter
        configMAX_PRIORITIES - 2, // Priority (high)
        &taskSafetyMonitor,     // Task handle
        0                       // Core 0
    );
    
    // Display task - Core 0, medium priority
    xTaskCreatePinnedToCore(
        displayTask,            // Task function
        "Display",              // Name
        8000,                   // Stack size
        NULL,                   // Parameter
        2,                      // Priority (medium)
        &taskDisplay,           // Task handle
        0                       // Core 0
    );
    
    // Communication task - Core 0, low priority
    xTaskCreatePinnedToCore(
        communicationTask,      // Task function
        "Communication",        // Name
        8000,                   // Stack size
        NULL,                   // Parameter
        1,                      // Priority (low)
        &taskCommunication,     // Task handle
        0                       // Core 0
    );
    
    // Wait for tasks to start
    delay(100);
    
    // Final system check
    if (!safetySystem->isSystemSafe()) {
        Serial.println("ERROR: System safety check failed");
        while (true) {
            delay(1000);
            Serial.println("SAFETY ERROR: System not safe to operate");
        }
    }
    
    Serial.println("System initialization complete");
    Serial.println("All safety systems operational");
    Serial.println("Ready for operation");
}

void loop() {
    // Main loop is empty - all work is done in FreeRTOS tasks
    // This prevents interfering with real-time motion control
    
    // Just feed the watchdog and yield to other tasks
    safetySystem->feedWatchdog();
    delay(100);
}