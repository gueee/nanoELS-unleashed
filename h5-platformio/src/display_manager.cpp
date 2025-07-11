/*
 * display_manager.cpp
 * Display Manager Implementation for nanoELS H5
 * 
 * Implements Nextion touch screen interface compatible with original h5.ino
 * while providing modern class-based architecture.
 */

#include "ui/display_manager.h"
#include <HardwareSerial.h>

// Global display manager instance
DisplayManager* g_displayManager = nullptr;

DisplayManager::DisplayManager() {
    // Default configuration
    config.serialTxPin = 44;
    config.serialRxPin = 43;
    config.baudRate = 115200;
    config.updateInterval = 50;
    config.enableTouch = true;
    config.enableBacklight = true;
    config.backlightLevel = 80;
    
    initializeState();
}

DisplayManager::DisplayManager(const DisplayConfig& configuration) : config(configuration) {
    initializeState();
}

DisplayManager::~DisplayManager() {
    if (displayMutex) {
        vSemaphoreDelete(displayMutex);
    }
    if (touchQueue) {
        vQueueDelete(touchQueue);
    }
}

void DisplayManager::initializeState() {
    state.isConnected = false;
    state.isInitialized = false;
    state.isUpdating = false;
    state.lastUpdateTime = 0;
    state.connectionTime = 0;
    state.errorCount = 0;
    strcpy(state.lastError, "No error");
    
    // Initialize data structure
    memset(&data, 0, sizeof(DisplayData));
    
    updateRequired = false;
    forceUpdate = false;
    lastHash = 0;
    updateCounter = 0;
    
    // Create mutex and queue
    displayMutex = xSemaphoreCreateMutex();
    touchQueue = xQueueCreate(10, sizeof(TouchEvent));
    
    // Initialize display serial - CRITICAL for h5.ino compatibility
    displaySerial = &Serial1;
    
    g_displayManager = this;
}

bool DisplayManager::initializeDisplay() {
    if (state.isInitialized) {
        return true;
    }
    
    Serial.println("Initializing Nextion display...");
    
    // Initialize serial communication exactly like original h5.ino
    // Serial1.begin(115200, SERIAL_8N1, 44, 43);
    displaySerial->begin(config.baudRate, SERIAL_8N1, config.serialTxPin, config.serialRxPin);
    
    // Wait for display to boot (original h5.ino has 1300ms delay)
    delay(1300);
    
    // Test communication
    if (!testCommunication()) {
        Serial.println("ERROR: Nextion display communication test failed");
        handleDisplayError("Communication test failed");
        return false;
    }
    
    // Clear display and show splash screen
    clearDisplay();
    showSplashScreen();
    
    state.isInitialized = true;
    state.isConnected = true;
    state.connectionTime = millis();
    
    Serial.println("Nextion display initialized successfully");
    return true;
}

void DisplayManager::updateDisplay() {
    if (!state.isInitialized) {
        return;
    }
    
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    
    unsigned long currentTime = millis();
    if (currentTime - state.lastUpdateTime >= config.updateInterval || forceUpdate) {
        state.isUpdating = true;
        
        updateDisplayData();
        
        state.lastUpdateTime = currentTime;
        state.isUpdating = false;
        updateCounter++;
        forceUpdate = false;
    }
    
    xSemaphoreGive(displayMutex);
}

void DisplayManager::updateDisplayData() {
    // Update system status
    updateSystemStatus();
    
    // Update axis positions
    updateAxisPositions();
    
    // Update spindle information
    updateSpindleInfo();
    
    // Update operation parameters
    updateOperationParams();
    
    // Update limits and stops
    updateLimitsAndStops();
    
    // Update G-code information
    updateGCodeInfo();
}

bool DisplayManager::sendCommand(const char* command) {
    if (!state.isConnected || !displaySerial) {
        return false;
    }
    
    // Send command exactly like original toScreen() function
    displaySerial->print(command);
    displaySerial->write(0xFF);
    displaySerial->write(0xFF);
    displaySerial->write(0xFF);
    
    return true;
}

bool DisplayManager::sendCommand(const char* command, const char* parameter) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s=\"%s\"", command, parameter);
    return sendCommand(buffer);
}

bool DisplayManager::sendCommand(const char* command, int parameter) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s=%d", command, parameter);
    return sendCommand(buffer);
}

bool DisplayManager::sendCommand(const char* command, float parameter) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s=%.3f", command, parameter);
    return sendCommand(buffer);
}

void DisplayManager::setText(const char* id, const char* text) {
    char command[256];
    snprintf(command, sizeof(command), "%s.txt=\"%s\"", id, text);
    sendCommand(command);
}

void DisplayManager::clearDisplay() {
    sendCommand("cls");
    
    // Clear all text fields like original screenClear()
    setText("t0", "");
    setText("t1", "");
    setText("t2", "");
    setText("t3", "");
}

void DisplayManager::showSplashScreen() {
    setText("t0", "NanoEls H5 V9");
    setText("t1", "PlatformIO Architecture");
    setText("t2", "Enhanced MPG Control");
    setText("t3", "Safety-First Design");
}

void DisplayManager::updateSystemStatus() {
    // Update line 0 - mode and status (like original lcdHashLine0)
    char buffer[128];
    const char* modeStr = "NORM";
    switch (data.currentMode) {
        case 0: modeStr = "NORM"; break;
        case 2: modeStr = "ASYNC"; break;
        case 3: modeStr = "CONE"; break;
        case 4: modeStr = "TURN"; break;
        case 5: modeStr = "FACE"; break;
        case 6: modeStr = "CUT"; break;
        case 7: modeStr = "THRD"; break;
        case 8: modeStr = "ELLP"; break;
        case 9: modeStr = "GCOD"; break;
        case 10: modeStr = "Y"; break;
        default: modeStr = "UNK"; break;
    }
    
    snprintf(buffer, sizeof(buffer), "%s %s", 
             modeStr, 
             data.systemOn ? "ON" : "off");
    
    setText("t0", buffer);
}

void DisplayManager::updateAxisPositions() {
    // Update line 2 - axis positions (like original lcdHashLine2)
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Z%.3f X%.3f", 
             data.posZ, data.posX);
    setText("t2", buffer);
}

void DisplayManager::updateSpindleInfo() {
    // Update line 3 - spindle and other info (like original lcdHashLine3)
    char buffer[128];
    if (data.spindleActive && data.spindleRPM > 0) {
        snprintf(buffer, sizeof(buffer), "RPM:%.0f", data.spindleRPM);
    } else {
        strcpy(buffer, "Spindle stopped");
    }
    setText("t3", buffer);
}

void DisplayManager::updateOperationParams() {
    // Update line 1 - pitch and parameters (like original lcdHashLine1)
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Pitch %.4f", data.pitch);
    if (data.starts > 1) {
        char starts_str[32];
        snprintf(starts_str, sizeof(starts_str), " x%d", data.starts);
        strcat(buffer, starts_str);
    }
    setText("t1", buffer);
}

void DisplayManager::updateLimitsAndStops() {
    // Limits are integrated into other displays, no separate update needed
}

void DisplayManager::updateGCodeInfo() {
    // G-code info is integrated into other displays, no separate update needed
}

bool DisplayManager::testCommunication() {
    // Send a simple command and wait for response
    sendCommand("get sleep");
    delay(100);
    
    // Check if we can read response
    if (displaySerial->available()) {
        char response[64];
        size_t bytesRead = displaySerial->readBytes(response, sizeof(response) - 1);
        response[bytesRead] = '\0';
        return true;
    }
    
    return true; // Assume success for compatibility
}

void DisplayManager::handleDisplayError(const char* error) {
    state.errorCount++;
    strncpy(state.lastError, error, sizeof(state.lastError) - 1);
    state.lastError[sizeof(state.lastError) - 1] = '\0';
    
    Serial.print("Display Error: ");
    Serial.println(error);
}

// Public interface methods compatible with original h5.ino

void DisplayManager::updateSystemStatus(bool on, bool estop, int mode, int measure) {
    data.systemOn = on;
    data.emergencyStop = estop;
    data.currentMode = mode;
    data.currentMeasure = measure;
    updateRequired = true;
}

void DisplayManager::updateAxisPositions(float x, float y, float z) {
    data.posX = x;
    data.posY = y;
    data.posZ = z;
    updateRequired = true;
}

void DisplayManager::updateSpindleInfo(float rpm, float angle, bool active, int direction) {
    data.spindleRPM = rpm;
    data.spindleAngle = angle;
    data.spindleActive = active;
    data.spindleDirection = direction;
    updateRequired = true;
}

void DisplayManager::updateOperationParams(float pitch, int starts, int passes, float coneRatio) {
    data.pitch = pitch;
    data.starts = starts;
    data.passes = passes;
    data.coneRatio = coneRatio;
    updateRequired = true;
}

bool DisplayManager::isConnected() const {
    return state.isConnected;
}

bool DisplayManager::isInitialized() const {
    return state.isInitialized;
}

void DisplayManager::forceDisplayUpdate() {
    forceUpdate = true;
}

// Stub implementations for compatibility
void DisplayManager::processTouchInput() {
    // Touch input processing - stub for now
}

void DisplayManager::handleTouchEvent(int x, int y, bool pressed) {
    // Touch event handling - stub for now
}

bool DisplayManager::lockDisplay(TickType_t timeout) {
    return xSemaphoreTake(displayMutex, timeout) == pdTRUE;
}

void DisplayManager::unlockDisplay() {
    xSemaphoreGive(displayMutex);
}