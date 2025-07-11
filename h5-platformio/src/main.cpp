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
#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <driver/pcnt.h>
#include <Preferences.h>
#include <PS2KeyAdvanced.h>
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
TaskHandle_t taskKeypad = NULL;

// Global variables from original h5.ino
bool isOn = false;
bool nextIsOn = false;
bool nextIsOnFlag = false;
unsigned long resetMillis = 0;
int emergencyStop = 0;

long dupr = 0; // pitch, tenth of a micron per rotation
long savedDupr = 0; // dupr saved in Preferences
long nextDupr = dupr; // dupr value that should be applied asap
bool nextDuprFlag = false; // whether nextDupr requires attention

SemaphoreHandle_t motionMutex; // controls blocks of code where variables affecting the motion loop() are changed

int starts = 1; // number of starts in a multi-start thread
int savedStarts = 0; // starts saved in Preferences
int nextStarts = starts; // number of starts that should be used asap
bool nextStartsFlag = false; // whether nextStarts requires attention

// Mode and operation variables
int mode = MODE_NORMAL;
int savedMode = 0;
int measure = MEASURE_METRIC;
int savedMeasure = 0;
float coneRatio = 1.0;
float savedConeRatio = 0.0;
int turnPasses = 1;
int savedTurnPasses = 0;
bool auxForward = true;
bool savedAuxForward = true;
long moveStep = MOVE_STEP_1;
long savedMoveStep = 0;

// Enhanced MPG velocity control parameters
const float MPG_VELOCITY_SCALE = 0.5; // Scale factor for MPG velocity control
const float MPG_ACCELERATION_SCALE = 2.0; // Acceleration scaling for MPG
const int MPG_VELOCITY_SAMPLES = 10; // Number of samples to average for velocity calculation
const unsigned long MPG_VELOCITY_TIMEOUT_MS = 100; // Timeout for velocity calculation

// MPG velocity tracking structures
struct MPGVelocityTracker {
  unsigned long lastPulseTime[MPG_VELOCITY_SAMPLES];
  int pulseCount[MPG_VELOCITY_SAMPLES];
  int sampleIndex;
  float currentVelocity;
  float targetVelocity;
  float acceleration;
  bool isActive;
};

MPGVelocityTracker zMpgTracker;
MPGVelocityTracker xMpgTracker;

// Initialize MPG velocity trackers
void initMPGVelocityTrackers() {
  memset(&zMpgTracker, 0, sizeof(MPGVelocityTracker));
  memset(&xMpgTracker, 0, sizeof(MPGVelocityTracker));
}

// Calculate MPG velocity based on pulse frequency
float calculateMPGVelocity(MPGVelocityTracker* tracker, int pulseDelta) {
  unsigned long currentTime = millis();
  
  if (pulseDelta != 0) {
    tracker->lastPulseTime[tracker->sampleIndex] = currentTime;
    tracker->pulseCount[tracker->sampleIndex] = pulseDelta;
    tracker->sampleIndex = (tracker->sampleIndex + 1) % MPG_VELOCITY_SAMPLES;
    tracker->isActive = true;
  }
  
  // Calculate average velocity from recent samples
  float totalVelocity = 0;
  int validSamples = 0;
  
  for (int i = 0; i < MPG_VELOCITY_SAMPLES; i++) {
    if (tracker->lastPulseTime[i] > 0 && 
        (currentTime - tracker->lastPulseTime[i]) < MPG_VELOCITY_TIMEOUT_MS) {
      float timeDiff = (currentTime - tracker->lastPulseTime[i]) / 1000.0f;
      if (timeDiff > 0) {
        totalVelocity += abs(tracker->pulseCount[i]) / timeDiff;
        validSamples++;
      }
    }
  }
  
  if (validSamples > 0) {
    tracker->currentVelocity = totalVelocity / validSamples;
  } else {
    tracker->currentVelocity = 0;
    tracker->isActive = false;
  }
  
  return tracker->currentVelocity;
}

// Enhanced MPG movement with velocity control
void moveAxisWithMPGVelocity(AxisController* a, MPGVelocityTracker* tracker, int pulseDelta, bool direction) {
  float velocity = calculateMPGVelocity(tracker, pulseDelta);
  
  // Scale velocity for more natural feel
  float scaledVelocity = velocity * MPG_VELOCITY_SCALE;
  
  // Calculate movement based on velocity rather than just pulse count
  float movement = 0;
  if (pulseDelta != 0) {
    // Direct pulse-based movement for immediate response
    movement = pulseDelta / PULSE_PER_REVOLUTION * a->getConfiguration().motorSteps;
  } else if (tracker->isActive && scaledVelocity > 0.1) {
    // Velocity-based movement for smooth control
    movement = scaledVelocity * a->getConfiguration().motorSteps / PULSE_PER_REVOLUTION;
  }
  
  if (movement != 0) {
    int delta = round(movement * (direction ? 1 : -1));
    long posCopy = a->getCurrentPosition();
    
    // Apply limits
    if (posCopy + delta > a->getLeftStop()) {
      delta = a->getLeftStop() - posCopy;
    } else if (posCopy + delta < a->getRightStop()) {
      delta = a->getRightStop() - posCopy;
    }
    
    if (delta != 0) {
      // Adjust speed based on velocity for more responsive feel
      float speedMultiplier = min(2.0f, 1.0f + scaledVelocity / 10.0f);
      long newSpeed = min(a->getConfiguration().speedManualMove * speedMultiplier, 
                         a->getConfiguration().speedManualMove * 2);
      
      a->moveRelative(delta, newSpeed);
    }
  }
}

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

// Keypad task - runs on Core 0
void keypadTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms cycle time
    
    while (true) {
        // Process PS2 keyboard input
        if (communicationManager) {
            communicationManager->processKeyboardInput();
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
    EncoderConfig encoderConfig = {
        .encoderA = ENC_A,
        .encoderB = ENC_B,
        .encoderPPR = ENCODER_PPR,
        .encoderBacklash = ENCODER_BACKLASH,
        .encoderFilter = ENCODER_FILTER,
        .pcntUnit = PCNT_UNIT_0,
        .pcntLimit = PCNT_LIM,
        .pcntClear = PCNT_CLEAR
    };
    
    spindleEncoder = new SpindleEncoder(encoderConfig);
    if (!spindleEncoder->initializeEncoder()) {
        Serial.println("ERROR: Failed to initialize spindle encoder");
        safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
    }
    
    // Initialize axis controllers
    AxisConfig configZ = {
        .name = 'Z',
        .active = true,
        .rotational = false,
        .stepPin = Z_STEP,
        .directionPin = Z_DIR,
        .enablePin = Z_ENA,
        .pulseAPin = Z_PULSE_A,
        .pulseBPin = Z_PULSE_B,
        .limitMinPin = -1,
        .limitMaxPin = -1,
        .motorSteps = MOTOR_STEPS_Z,
        .screwPitch = SCREW_Z_DU,
        .backlashSteps = BACKLASH_DU_Z * MOTOR_STEPS_Z / SCREW_Z_DU,
        .speedStart = SPEED_START_Z,
        .speedMax = SPEED_MANUAL_MOVE_Z * 2,
        .acceleration = ACCELERATION_Z,
        .maxTravelMm = MAX_TRAVEL_MM_Z,
        .invertDirection = INVERT_Z,
        .invertEnable = INVERT_Z_ENABLE,
        .needsRest = NEEDS_REST_Z
    };
    
    axisZ = new AxisController('Z', configZ);
    if (!axisZ->initializeAxis()) {
        Serial.println("ERROR: Failed to initialize Z axis");
        safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
    }
    
    AxisConfig configX = {
        .name = 'X',
        .active = true,
        .rotational = false,
        .stepPin = X_STEP,
        .directionPin = X_DIR,
        .enablePin = X_ENA,
        .pulseAPin = X_PULSE_A,
        .pulseBPin = X_PULSE_B,
        .limitMinPin = -1,
        .limitMaxPin = -1,
        .motorSteps = MOTOR_STEPS_X,
        .screwPitch = SCREW_X_DU,
        .backlashSteps = BACKLASH_DU_X * MOTOR_STEPS_X / SCREW_X_DU,
        .speedStart = SPEED_START_X,
        .speedMax = SPEED_MANUAL_MOVE_X * 2,
        .acceleration = ACCELERATION_X,
        .maxTravelMm = MAX_TRAVEL_MM_X,
        .invertDirection = INVERT_X,
        .invertEnable = INVERT_X_ENABLE,
        .needsRest = NEEDS_REST_X
    };
    
    axisX = new AxisController('X', configX);
    if (!axisX->initializeAxis()) {
        Serial.println("ERROR: Failed to initialize X axis");
        safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
    }
    
    // Initialize Y axis if enabled
    if (ACTIVE_Y) {
        AxisConfig configY = {
            .name = 'Y',
            .active = true,
            .rotational = ROTARY_Y,
            .stepPin = Y_STEP,
            .directionPin = Y_DIR,
            .enablePin = Y_ENA,
            .pulseAPin = Y_PULSE_A,
            .pulseBPin = Y_PULSE_B,
            .limitMinPin = -1,
            .limitMaxPin = -1,
            .motorSteps = MOTOR_STEPS_Y,
            .screwPitch = SCREW_Y_DU,
            .backlashSteps = BACKLASH_DU_Y * MOTOR_STEPS_Y / SCREW_Y_DU,
            .speedStart = SPEED_START_Y,
            .speedMax = SPEED_MANUAL_MOVE_Y * 2,
            .acceleration = ACCELERATION_Y,
            .maxTravelMm = MAX_TRAVEL_MM_Y,
            .invertDirection = INVERT_Y,
            .invertEnable = INVERT_Y_ENABLE,
            .needsRest = NEEDS_REST_Y
        };
        
        axisY = new AxisController('Y', configY);
        if (!axisY->initializeAxis()) {
            Serial.println("ERROR: Failed to initialize Y axis");
            safetySystem->triggerEmergencyStop(ESTOP_HARDWARE_FAILURE);
        }
    }
    
    // Initialize motion planner
    motionPlanner = new MotionPlanner(axisZ, axisX, axisY, spindleEncoder, safetySystem);
    
    // Initialize display manager
    DisplayConfig displayConfig = {
        .serialTxPin = 44,
        .serialRxPin = 43,
        .baudRate = 115200,
        .updateInterval = 50,
        .enableTouch = true,
        .enableBacklight = true,
        .backlightLevel = 80
    };
    
    displayManager = new DisplayManager(displayConfig);
    if (!displayManager->initializeDisplay()) {
        Serial.println("WARNING: Failed to initialize display");
    }
    
    // Initialize communication manager
    CommunicationConfig commConfig = {
        .ssid = SSID,
        .password = PASSWORD,
        .hostname = "nanoels-h5",
        .wifiTimeout = 10000,
        .webServerPort = 80,
        .webSocketPort = 81,
        .maxConnections = 4,
        .bufferSize = 100000,
        .keyboardDataPin = KEY_DATA,
        .keyboardClockPin = KEY_CLOCK,
        .keyboardEnabled = true,
        .keyboardTimeout = 1000,
        .enableWebInterface = WIFI_ENABLED,
        .enableWebSocket = true,
        .enableSerialDebug = true,
        .updateInterval = 20
    };
    
    communicationManager = new CommunicationManager(commConfig);
    if (!communicationManager->initializeCommunication()) {
        Serial.println("WARNING: Failed to initialize communication");
    }
    
    // Initialize MPG velocity trackers
    initMPGVelocityTrackers();
    
    // Create motion mutex
    motionMutex = xSemaphoreCreateMutex();
    
    // Load saved settings
    loadSavedSettings();
    
    // Initialize file system
    if (LittleFS.begin(true)) {
        Serial.println("File system initialized");
    }
    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(motionControlTask, "motionControl", 10000, NULL, 2, &taskMotionControl, 1);
    xTaskCreatePinnedToCore(safetyMonitorTask, "safetyMonitor", 10000, NULL, 3, &taskSafetyMonitor, 0);
    xTaskCreatePinnedToCore(displayTask, "display", 10000, NULL, 1, &taskDisplay, 0);
    xTaskCreatePinnedToCore(communicationTask, "communication", 10000, NULL, 1, &taskCommunication, 0);
    xTaskCreatePinnedToCore(keypadTask, "keypad", 10000, NULL, 1, &taskKeypad, 0);
    
    Serial.println("nanoELS H5 initialization complete");
}

void loop() {
    // Main loop is handled by FreeRTOS tasks
    // This function is kept for compatibility but should not be used for real-time operations
    vTaskDelay(pdMS_TO_TICKS(100));
}

// Load saved settings from preferences
void loadSavedSettings() {
    Preferences pref;
    pref.begin(CONFIG_NAMESPACE);
    
    // Load system settings
    isOn = false;
    savedDupr = dupr = pref.getLong("dupr", 0);
    savedStarts = starts = min(STARTS_MAX, max(static_cast<int32_t>(1), pref.getInt("starts", 1)));
    savedMode = mode = pref.getInt("mode", MODE_NORMAL);
    savedMeasure = measure = pref.getInt("measure", MEASURE_METRIC);
    savedConeRatio = coneRatio = pref.getFloat("coneRatio", 1.0);
    savedTurnPasses = turnPasses = pref.getInt("turnPasses", 1);
    savedAuxForward = auxForward = pref.getBool("auxForward", true);
    savedMoveStep = moveStep = pref.getLong("moveStep", MOVE_STEP_1);
    
    // Load axis settings
    if (axisZ) {
        axisZ->setCurrentPosition(pref.getLong("posZ", 0));
        axisZ->setLeftStop(pref.getLong("leftStopZ", LONG_MAX));
        axisZ->setRightStop(pref.getLong("rightStopZ", LONG_MIN));
        axisZ->enableAxis(!pref.getBool("disabledZ", false));
    }
    
    if (axisX) {
        axisX->setCurrentPosition(pref.getLong("posX", 0));
        axisX->setLeftStop(pref.getLong("leftStopX", LONG_MAX));
        axisX->setRightStop(pref.getLong("rightStopX", LONG_MIN));
        axisX->enableAxis(!pref.getBool("disabledX", false));
    }
    
    if (axisY && ACTIVE_Y) {
        axisY->setCurrentPosition(pref.getLong("posY", 0));
        axisY->setLeftStop(pref.getLong("leftStopY", LONG_MAX));
        axisY->setRightStop(pref.getLong("rightStopY", LONG_MIN));
        axisY->enableAxis(!pref.getBool("disabledY", false));
    }
    
    // Load spindle settings
    if (spindleEncoder) {
        spindleEncoder->setEncoderPosition(pref.getLong("spindlePos", 0));
    }
    
    pref.end();
}

// Save settings to preferences
void saveSettings() {
    Preferences pref;
    pref.begin(CONFIG_NAMESPACE);
    
    // Save system settings
    pref.putLong("dupr", dupr);
    pref.putInt("starts", starts);
    pref.putInt("mode", mode);
    pref.putInt("measure", measure);
    pref.putFloat("coneRatio", coneRatio);
    pref.putInt("turnPasses", turnPasses);
    pref.putBool("auxForward", auxForward);
    pref.putLong("moveStep", moveStep);
    
    // Save axis settings
    if (axisZ) {
        pref.putLong("posZ", axisZ->getCurrentPosition());
        pref.putLong("leftStopZ", axisZ->getLeftStop());
        pref.putLong("rightStopZ", axisZ->getRightStop());
        pref.putBool("disabledZ", !axisZ->isEnabled());
    }
    
    if (axisX) {
        pref.putLong("posX", axisX->getCurrentPosition());
        pref.putLong("leftStopX", axisX->getLeftStop());
        pref.putLong("rightStopX", axisX->getRightStop());
        pref.putBool("disabledX", !axisX->isEnabled());
    }
    
    if (axisY && ACTIVE_Y) {
        pref.putLong("posY", axisY->getCurrentPosition());
        pref.putLong("leftStopY", axisY->getLeftStop());
        pref.putLong("rightStopY", axisY->getRightStop());
        pref.putBool("disabledY", !axisY->isEnabled());
    }
    
    // Save spindle settings
    if (spindleEncoder) {
        pref.putLong("spindlePos", spindleEncoder->getCurrentPosition());
    }
    
    pref.end();
}

// Apply pending settings changes
void applySettings() {
    if (nextIsOnFlag) {
        isOn = nextIsOn;
        nextIsOnFlag = false;
    }
    
    if (nextDuprFlag) {
        dupr = nextDupr;
        nextDuprFlag = false;
    }
    
    if (nextStartsFlag) {
        starts = nextStarts;
        nextStartsFlag = false;
    }
}

// Emergency stop handling
void handleEmergencyStop() {
    if (emergencyStop != ESTOP_NONE) {
        // Stop all motion
        if (axisZ) axisZ->stopMotion(true);
        if (axisX) axisX->stopMotion(true);
        if (axisY) axisY->stopMotion(true);
        
        // Disable all axes
        if (axisZ) axisZ->enableAxis(false);
        if (axisX) axisX->enableAxis(false);
        if (axisY) axisY->enableAxis(false);
        
        // Turn off system
        isOn = false;
        
        // Update display
        if (displayManager) {
            displayManager->updateSystemStatus(false, true, mode, measure);
        }
    }
}