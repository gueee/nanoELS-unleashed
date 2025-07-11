/*
 * MPG Velocity Control Test Program (PRIVATE HARDWARE VERSION)
 *
 * This is a customized copy of MPG_TEST.ino for the user's specific hardware setup.
 * All pin assignments, PPR, and key mappings are taken from Private/h5.ino.
 */

#include <Arduino.h>
#include <driver/pcnt.h>

// === Hardware-specific settings from Private/h5.ino ===
#define ENCODER_PPR 600
#define PULSE_PER_REVOLUTION 600
#define ENC_A 13
#define ENC_B 14
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
#define KEY_DATA 37
#define KEY_CLOCK 36
// ... (add more as needed)

#define INVERT_Z_ENABLE true
#define INVERT_X_ENABLE true

// If you want to test Z axis, use Z_* pins. For X axis, use X_* pins.
// For this test, we're using Z axis pins and INVERT_Z_ENABLE
const int TEST_PULSE_PIN = Z_PULSE_A; // Connect MPG encoder to this pin
const int TEST_DIR_PIN = Z_DIR;       // Direction pin
const int TEST_STEP_PIN = Z_STEP;     // Step pin
const int TEST_ENA_PIN = Z_ENA;       // Enable pin

#define PCNT_UNIT      PCNT_UNIT_1
#define PCNT_PULSE_PIN Z_PULSE_A
#define PCNT_CTRL_PIN  Z_PULSE_B
#define PCNT_H_LIM     31000
#define PCNT_L_LIM    -31000
#define PCNT_FILTER    1

// MPG velocity control parameters (same as in main code)
const float MPG_VELOCITY_SCALE = 0.5;
const float MPG_ACCELERATION_SCALE = 2.0;
const int MPG_VELOCITY_SAMPLES = 10;
const unsigned long MPG_VELOCITY_TIMEOUT_MS = 100;

// MPG velocity tracking structure
struct MPGVelocityTracker {
  unsigned long lastPulseTime[MPG_VELOCITY_SAMPLES];
  int pulseCount[MPG_VELOCITY_SAMPLES];
  int sampleIndex;
  float currentVelocity;
  float targetVelocity;
  float acceleration;
  bool isActive;
};

MPGVelocityTracker mpgTracker;

// Test variables
volatile int pulseCount = 0;
volatile unsigned long lastPulseTime = 0;
bool testRunning = false;
unsigned long testStartTime = 0;

#define NEXTION_TX 44
#define NEXTION_RX 43

void setup() {
  Serial.begin(115200);
  Serial.println("MPG Velocity Control Test Program (PRIVATE HARDWARE)");
  Serial.println("=================================");

  // Nextion display boot fix
  Serial1.begin(115200, SERIAL_8N1, NEXTION_TX, NEXTION_RX);
  delay(1300);
  Serial1.print("t0.txt=\"MPG TEST\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  Serial1.print("t1.txt=\"Ready\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  Serial1.print("t2.txt=\"\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
  Serial1.print("t3.txt=\"\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);

  // Initialize pins
  pinMode(TEST_DIR_PIN, OUTPUT);
  pinMode(TEST_STEP_PIN, OUTPUT);
  pinMode(TEST_ENA_PIN, OUTPUT);
  
  // Initialize MPG tracker
  memset(&mpgTracker, 0, sizeof(MPGVelocityTracker));
  
  // Initialize PCNT for MPG input (quadrature)
  pcnt_config_t pcntConfig = {};
  pcntConfig.pulse_gpio_num = PCNT_PULSE_PIN;
  pcntConfig.ctrl_gpio_num = PCNT_CTRL_PIN;
  pcntConfig.channel = PCNT_CHANNEL_0;
  pcntConfig.unit = PCNT_UNIT;
  pcntConfig.pos_mode = PCNT_COUNT_INC;
  pcntConfig.neg_mode = PCNT_COUNT_DEC;
  pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.counter_h_lim = PCNT_H_LIM;
  pcntConfig.counter_l_lim = PCNT_L_LIM;
  pcnt_unit_config(&pcntConfig);
  pcnt_set_filter_value(PCNT_UNIT, PCNT_FILTER);
  pcnt_filter_enable(PCNT_UNIT);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

  Serial.println("Test program ready. Commands:");
  Serial.println("  'start' - Start velocity test");
  Serial.println("  'stop'  - Stop test");
  Serial.println("  'stats' - Show current statistics");
  Serial.println("  'reset' - Reset all counters");
}

int16_t lastPCNT = 0;
unsigned long lastUpdate = 0;

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "start") {
      startTest();
    } else if (command == "stop") {
      stopTest();
    } else if (command == "stats") {
      showStats();
    } else if (command == "reset") {
      resetTest();
    }
  }
  
  // Update velocity calculation
  if (testRunning) {
    updateVelocityPCNT();
    
    // Simulate motor movement based on velocity
    if (mpgTracker.currentVelocity > 0.1) {
      simulateMotorMovement();
    }
  }
  
  delay(10); // Small delay to prevent overwhelming the serial output
}

void updateVelocityPCNT() {
  unsigned long currentTime = millis();
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  int delta = count - lastPCNT;
  lastPCNT = count;

  // Store delta as a sample for velocity calculation
  if (delta != 0) {
    mpgTracker.lastPulseTime[mpgTracker.sampleIndex] = currentTime;
    mpgTracker.pulseCount[mpgTracker.sampleIndex] = delta;
    mpgTracker.sampleIndex = (mpgTracker.sampleIndex + 1) % MPG_VELOCITY_SAMPLES;
    mpgTracker.isActive = true;
  }

  // Calculate average velocity from recent samples
  float totalVelocity = 0;
  int validSamples = 0;
  
  for (int i = 0; i < MPG_VELOCITY_SAMPLES; i++) {
    if (mpgTracker.lastPulseTime[i] > 0 && 
        (currentTime - mpgTracker.lastPulseTime[i]) < MPG_VELOCITY_TIMEOUT_MS) {
      float timeDiff = (currentTime - mpgTracker.lastPulseTime[i]) / 1000.0f;
      if (timeDiff > 0) {
        totalVelocity += abs(mpgTracker.pulseCount[i]) / timeDiff;
        validSamples++;
      }
    }
  }
  
  if (validSamples > 0) {
    mpgTracker.currentVelocity = totalVelocity / validSamples;
  } else {
    mpgTracker.currentVelocity = 0;
    mpgTracker.isActive = false;
  }

  // Display live stats on Nextion
  defineStatsDisplay(delta);
}

void defineStatsDisplay(int delta) {
  char buf[64];
  snprintf(buf, sizeof(buf), "d=%d v=%.2f", delta, mpgTracker.currentVelocity);
  Serial1.print("t2.txt=\""); Serial1.print(buf); Serial1.print("\""); Serial1.write(0xFF); Serial1.write(0xFF); Serial1.write(0xFF);
}

void startTest() {
  Serial.println("Starting MPG velocity test...");
  testRunning = true;
  testStartTime = millis();
  resetTest();
}

void stopTest() {
  Serial.println("Stopping MPG velocity test...");
  testRunning = false;
  // Disable motor:
  digitalWrite(TEST_ENA_PIN, INVERT_Z_ENABLE ? HIGH : LOW);
}

void resetTest() {
  pulseCount = 0;
  lastPulseTime = 0;
  memset(&mpgTracker, 0, sizeof(MPGVelocityTracker));
  Serial.println("Counters reset");
}

void simulateMotorMovement() {
  // Calculate movement based on velocity
  float scaledVelocity = mpgTracker.currentVelocity * MPG_VELOCITY_SCALE;
  float movement = scaledVelocity * 100; // Scale for simulation
  
  if (movement > 1.0) {
    // Simulate motor steps
    // Enable motor:
    digitalWrite(TEST_ENA_PIN, INVERT_Z_ENABLE ? LOW : HIGH);
    digitalWrite(TEST_DIR_PIN, movement > 0 ? HIGH : LOW);
    
    for (int i = 0; i < abs(movement); i++) {
      digitalWrite(TEST_STEP_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(TEST_STEP_PIN, LOW);
      delayMicroseconds(100);
    }
  } else {
    // Disable motor:
    digitalWrite(TEST_ENA_PIN, INVERT_Z_ENABLE ? HIGH : LOW);
  }
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
  
  Serial.print("Current velocity: ");
  Serial.print(mpgTracker.currentVelocity, 2);
  Serial.println(" pulses/sec");
  
  Serial.print("Scaled velocity: ");
  Serial.print(mpgTracker.currentVelocity * MPG_VELOCITY_SCALE, 2);
  Serial.println(" (scaled)");
  
  Serial.print("MPG active: ");
  Serial.println(mpgTracker.isActive ? "Yes" : "No");
  
  Serial.print("Sample index: ");
  Serial.println(mpgTracker.sampleIndex);
  
  Serial.println("Recent pulse samples:");
  for (int i = 0; i < MPG_VELOCITY_SAMPLES; i++) {
    if (mpgTracker.lastPulseTime[i] > 0) {
      Serial.print("  [");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(mpgTracker.pulseCount[i]);
      Serial.print(" pulses at ");
      Serial.print(mpgTracker.lastPulseTime[i]);
      Serial.println(" ms");
    }
  }
  Serial.println("==================================");
} 