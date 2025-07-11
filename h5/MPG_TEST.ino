/*
 * MPG Velocity Control Test Program for H5 Hardware
 * 
 * This program tests the enhanced MPG velocity control functionality
 * on the actual nanoELS H5 machine with:
 * - Two MPGs (X and Z axes)
 * - Two stepper motors with inverted enable signals
 * - Nextion display showing real-time coordinates
 * - Original h5.tft display configuration
 * 
 * Features tested:
 * - Velocity calculation from pulse frequency
 * - Adaptive speed control based on encoder rotation speed
 * - Smooth acceleration/deceleration
 * - Natural feel simulation
 * - Real-time position display
 */

#include <Arduino.h>
#include <PS2KeyAdvanced.h>

// Hardware pin definitions from H5.ino
#define Z_ENA 41
#define Z_DIR 42
#define Z_STEP 35
#define Z_PULSE_A 18
#define Z_PULSE_B 8

#define X_ENA 16
#define X_DIR 15
#define X_STEP 7
#define X_PULSE_A 47
#define X_PULSE_B 21

// Nextion display communication
#define NEXTION_TX 44
#define NEXTION_RX 43

// Keyboard key codes from H5.ino
#define B_MODE_GEARS 97  // F1
#define B_MODE_TURN 98   // F2
#define B_MODE_FACE 99   // F3
#define B_MODE_CONE 100  // F4
#define B_OFF 27         // ESC

// MPG velocity control parameters
const float MPG_VELOCITY_SCALE = 0.5;
const float MPG_ACCELERATION_SCALE = 2.0;
const int MPG_VELOCITY_SAMPLES = 10;
const unsigned long MPG_VELOCITY_TIMEOUT_MS = 100;

// Hardware configuration from H5.ino
const long SCREW_Z_DU = 40000; // 4mm SFU1204 ball screw
const long MOTOR_STEPS_Z = 800;
const long SPEED_MANUAL_MOVE_Z = 8 * MOTOR_STEPS_Z;
const bool INVERT_Z = false;
const bool INVERT_Z_ENABLE = false; // Will be inverted in code

const long SCREW_X_DU = 40000; // 4mm SFU1204 ball screw
const long MOTOR_STEPS_X = 800;
const long SPEED_MANUAL_MOVE_X = 8 * MOTOR_STEPS_X;
const bool INVERT_X = true;
const bool INVERT_X_ENABLE = false; // Will be inverted in code

const float PULSE_PER_REVOLUTION = 600; // PPR of handwheels

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

struct Axis {
  long pos;
  long pendingPos;
  long leftStop;
  long rightStop;
  long motorSteps;
  long speedMax;
  long speedManualMove;
  bool enabled;
  bool continuous;
  MPGVelocityTracker mpgTracker;
};

Axis zAxis;
Axis xAxis;

// Test variables
volatile int zPulseCount = 0;
volatile int xPulseCount = 0;
volatile unsigned long zLastPulseTime = 0;
volatile unsigned long xLastPulseTime = 0;
bool testRunning = false;
unsigned long testStartTime = 0;

// Nextion display variables
String nextionCommand = "";
bool nextionReady = false;

// Keyboard interface
PS2KeyAdvanced keyboard;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, NEXTION_TX, NEXTION_RX);
  
  Serial.println("MPG Velocity Control Test Program for H5");
  Serial.println("========================================");
  
  // Initialize motor pins
  pinMode(Z_ENA, OUTPUT);
  pinMode(Z_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  pinMode(X_ENA, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_STEP, OUTPUT);
  
  // Initialize MPG encoder pins
  pinMode(Z_PULSE_A, INPUT_PULLUP);
  pinMode(Z_PULSE_B, INPUT_PULLUP);
  pinMode(X_PULSE_A, INPUT_PULLUP);
  pinMode(X_PULSE_B, INPUT_PULLUP);
  
  // Initialize axis structures
  initAxis(&zAxis, MOTOR_STEPS_Z, SPEED_MANUAL_MOVE_Z, SCREW_Z_DU, 'Z');
  initAxis(&xAxis, MOTOR_STEPS_X, SPEED_MANUAL_MOVE_X, SCREW_X_DU, 'X');
  
  // Attach interrupts for MPG encoders
  attachInterrupt(digitalPinToInterrupt(Z_PULSE_A), zPulseInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(X_PULSE_A), xPulseInterrupt, RISING);
  
  // Initialize keyboard
  keyboard.begin(KEY_DATA, KEY_CLOCK);
  
  // Initialize Nextion display
  initNextionDisplay();
  
  Serial.println("MPG Test Mode Ready!");
  Serial.println("Keyboard Commands:");
  Serial.println("  F1 (Gears) - Start/Stop test");
  Serial.println("  F2 (Turn)  - Show statistics");
  Serial.println("  F3 (Face)  - Reset counters");
  Serial.println("  F4 (Cone)  - Zero axes");
  Serial.println("  ESC         - Emergency stop");
}

void loop() {
  // Handle keyboard commands
  if (keyboard.available()) {
    int event = keyboard.read();
    int keyCode = event & 0xFF;
    bool isPress = !(event & PS2_BREAK);
    
    if (isPress) {
      processKeyboardCommand(keyCode);
    }
  }
  
  // Update velocity calculations and motor control
  if (testRunning) {
    updateMPGVelocity(&zAxis, zPulseCount);
    updateMPGVelocity(&xAxis, xPulseCount);
    
    // Control motors based on MPG velocity
    if (zAxis.mpgTracker.isActive && zAxis.mpgTracker.currentVelocity > 0.1) {
      moveAxisWithMPGVelocity(&zAxis);
    }
    
    if (xAxis.mpgTracker.isActive && xAxis.mpgTracker.currentVelocity > 0.1) {
      moveAxisWithMPGVelocity(&xAxis);
    }
    
    // Update display
    updateNextionDisplay();
  }
  
  delay(10);
}

void processKeyboardCommand(int keyCode) {
  switch (keyCode) {
    case B_MODE_GEARS:  // F1 - Start/Stop test
      if (testRunning) {
        stopTest();
      } else {
        startTest();
      }
      break;
      
    case B_MODE_TURN:   // F2 - Show statistics
      showStats();
      break;
      
    case B_MODE_FACE:   // F3 - Reset counters
      resetTest();
      break;
      
    case B_MODE_CONE:   // F4 - Zero axes
      zeroAxes();
      break;
      
    case B_OFF:         // ESC - Emergency stop
      stopTest();
      Serial.println("Emergency stop activated!");
      break;
  }
}

void initAxis(Axis* axis, long motorSteps, long speedManualMove, long screwDu, char name) {
  axis->pos = 0;
  axis->pendingPos = 0;
  axis->leftStop = LONG_MAX;
  axis->rightStop = LONG_MIN;
  axis->motorSteps = motorSteps;
  axis->speedMax = speedManualMove;
  axis->speedManualMove = speedManualMove;
  axis->enabled = true;
  axis->continuous = false;
  
  // Initialize MPG tracker
  memset(&axis->mpgTracker, 0, sizeof(MPGVelocityTracker));
  
  Serial.print("Initialized axis ");
  Serial.print(name);
  Serial.print(" with motor steps: ");
  Serial.println(motorSteps);
}

void zPulseInterrupt() {
  zPulseCount++;
  zLastPulseTime = micros();
}

void xPulseInterrupt() {
  xPulseCount++;
  xLastPulseTime = micros();
}

void startTest() {
  Serial.println("Starting MPG velocity test on actual hardware...");
  testRunning = true;
  testStartTime = millis();
  resetTest();
  
  // Enable motors
  digitalWrite(Z_ENA, !INVERT_Z_ENABLE); // Invert enable signal
  digitalWrite(X_ENA, !INVERT_X_ENABLE); // Invert enable signal
}

void stopTest() {
  Serial.println("Stopping MPG velocity test...");
  testRunning = false;
  
  // Disable motors
  digitalWrite(Z_ENA, INVERT_Z_ENABLE);
  digitalWrite(X_ENA, INVERT_X_ENABLE);
}

void resetTest() {
  zPulseCount = 0;
  xPulseCount = 0;
  zLastPulseTime = 0;
  xLastPulseTime = 0;
  
  memset(&zAxis.mpgTracker, 0, sizeof(MPGVelocityTracker));
  memset(&xAxis.mpgTracker, 0, sizeof(MPGVelocityTracker));
  
  Serial.println("Counters reset");
}

void zeroAxes() {
  zAxis.pos = 0;
  xAxis.pos = 0;
  Serial.println("Both axes zeroed");
  updateNextionDisplay();
}

void updateMPGVelocity(Axis* axis, int pulseDelta) {
  unsigned long currentTime = millis();
  
  if (pulseDelta != 0) {
    axis->mpgTracker.lastPulseTime[axis->mpgTracker.sampleIndex] = currentTime;
    axis->mpgTracker.pulseCount[axis->mpgTracker.sampleIndex] = pulseDelta;
    axis->mpgTracker.sampleIndex = (axis->mpgTracker.sampleIndex + 1) % MPG_VELOCITY_SAMPLES;
    axis->mpgTracker.isActive = true;
  }
  
  // Calculate average velocity from recent samples
  float totalVelocity = 0;
  int validSamples = 0;
  
  for (int i = 0; i < MPG_VELOCITY_SAMPLES; i++) {
    if (axis->mpgTracker.lastPulseTime[i] > 0 && 
        (currentTime - axis->mpgTracker.lastPulseTime[i]) < MPG_VELOCITY_TIMEOUT_MS) {
      float timeDiff = (currentTime - axis->mpgTracker.lastPulseTime[i]) / 1000.0f;
      if (timeDiff > 0) {
        totalVelocity += abs(axis->mpgTracker.pulseCount[i]) / timeDiff;
        validSamples++;
      }
    }
  }
  
  if (validSamples > 0) {
    axis->mpgTracker.currentVelocity = totalVelocity / validSamples;
  } else {
    axis->mpgTracker.currentVelocity = 0;
    axis->mpgTracker.isActive = false;
  }
}

void moveAxisWithMPGVelocity(Axis* axis) {
  float velocity = axis->mpgTracker.currentVelocity;
  float scaledVelocity = velocity * MPG_VELOCITY_SCALE;
  
  // Calculate movement based on velocity
  float movement = scaledVelocity * axis->motorSteps / PULSE_PER_REVOLUTION;
  
  if (movement > 0.1) {
    // Determine direction based on MPG rotation
    bool direction = (axis == &zAxis) ? (zPulseCount > 0) : (xPulseCount > 0);
    
    // Apply direction inversion
    if (axis == &zAxis && INVERT_Z) direction = !direction;
    if (axis == &xAxis && INVERT_X) direction = !direction;
    
    int delta = round(movement) * (direction ? 1 : -1);
    long posCopy = axis->pos + axis->pendingPos;
    
    // Apply limits
    if (posCopy + delta > axis->leftStop) {
      delta = axis->leftStop - posCopy;
    } else if (posCopy + delta < axis->rightStop) {
      delta = axis->rightStop - posCopy;
    }
    
    if (delta != 0) {
      // Adjust speed based on velocity for more responsive feel
      float speedMultiplier = min(2.0f, 1.0f + scaledVelocity / 10.0f);
      axis->speedMax = min(axis->speedManualMove * speedMultiplier, axis->speedManualMove * 2);
      
      // Move motor
      stepMotor(axis, delta);
      
      // Update position
      axis->pos += delta;
    }
  }
}

void stepMotor(Axis* axis, int steps) {
  if (steps == 0) return;
  
  // Set direction
  bool direction = steps > 0;
  if (axis == &zAxis) {
    digitalWrite(Z_DIR, direction ? HIGH : LOW);
  } else {
    digitalWrite(X_DIR, direction ? HIGH : LOW);
  }
  
  // Generate steps
  int stepPin = (axis == &zAxis) ? Z_STEP : X_STEP;
  int absSteps = abs(steps);
  
  for (int i = 0; i < absSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
  }
}

void initNextionDisplay() {
  Serial.println("Initializing Nextion display...");
  
  // Wait for Nextion to boot
  delay(1300);
  
  // Send initial commands to display
  sendNextionCommand("page 0");
  sendNextionCommand("t0.txt=\"MPG Test Mode\"");
  sendNextionCommand("t1.txt=\"Z: 0.000\"");
  sendNextionCommand("t2.txt=\"X: 0.000\"");
  sendNextionCommand("t3.txt=\"Z Vel: 0.0\"");
  sendNextionCommand("t4.txt=\"X Vel: 0.0\"");
  
  nextionReady = true;
  Serial.println("Nextion display initialized");
}

void updateNextionDisplay() {
  if (!nextionReady) return;
  
  // Update Z position (convert from steps to mm)
  float zPosMm = (float)zAxis.pos / zAxis.motorSteps * 4.0; // 4mm pitch
  String zPosStr = "Z: " + String(zPosMm, 3);
  sendNextionCommand("t1.txt=\"" + zPosStr + "\"");
  
  // Update X position
  float xPosMm = (float)xAxis.pos / xAxis.motorSteps * 4.0; // 4mm pitch
  String xPosStr = "X: " + String(xPosMm, 3);
  sendNextionCommand("t2.txt=\"" + xPosStr + "\"");
  
  // Update velocities
  String zVelStr = "Z Vel: " + String(zAxis.mpgTracker.currentVelocity, 1);
  sendNextionCommand("t3.txt=\"" + zVelStr + "\"");
  
  String xVelStr = "X Vel: " + String(xAxis.mpgTracker.currentVelocity, 1);
  sendNextionCommand("t4.txt=\"" + xVelStr + "\"");
}

void sendNextionCommand(String command) {
  Serial1.print(command);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void showStats() {
  Serial.println("=== MPG Velocity Test Statistics ===");
  Serial.print("Test running: ");
  Serial.println(testRunning ? "Yes" : "No");
  
  if (testRunning) {
    unsigned long runTime = millis() - testStartTime;
    Serial.print("Run time: ");
    Serial.print(runTime / 1000.0, 1);
    Serial.println(" seconds");
  }
  
  Serial.println("Z Axis:");
  Serial.print("  Position: ");
  Serial.print(zAxis.pos);
  Serial.println(" steps");
  Serial.print("  Velocity: ");
  Serial.print(zAxis.mpgTracker.currentVelocity, 2);
  Serial.println(" pulses/sec");
  Serial.print("  MPG active: ");
  Serial.println(zAxis.mpgTracker.isActive ? "Yes" : "No");
  
  Serial.println("X Axis:");
  Serial.print("  Position: ");
  Serial.print(xAxis.pos);
  Serial.println(" steps");
  Serial.print("  Velocity: ");
  Serial.print(xAxis.mpgTracker.currentVelocity, 2);
  Serial.println(" pulses/sec");
  Serial.print("  MPG active: ");
  Serial.println(xAxis.mpgTracker.isActive ? "Yes" : "No");
  
  Serial.println("==================================");
} 