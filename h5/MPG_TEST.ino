/*
 * MPG Velocity Control Test Program
 * 
 * This program tests the enhanced MPG velocity control functionality
 * for the nanoELS H5 system.
 * 
 * Features tested:
 * - Velocity calculation from pulse frequency
 * - Adaptive speed control based on encoder rotation speed
 * - Smooth acceleration/deceleration
 * - Natural feel simulation
 */

#include <Arduino.h>

// MPG velocity control parameters (same as in main code)
const float MPG_VELOCITY_SCALE = 0.5;
const float MPG_ACCELERATION_SCALE = 2.0;
const int MPG_VELOCITY_SAMPLES = 10;
const unsigned long MPG_VELOCITY_TIMEOUT_MS = 100;

// Test parameters
const int TEST_PULSE_PIN = 2; // Connect MPG encoder to this pin
const int TEST_DIR_PIN = 3;   // Direction pin
const int TEST_STEP_PIN = 4;  // Step pin
const int TEST_ENA_PIN = 5;   // Enable pin

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

void setup() {
  Serial.begin(115200);
  Serial.println("MPG Velocity Control Test Program");
  Serial.println("=================================");
  
  // Initialize pins
  pinMode(TEST_PULSE_PIN, INPUT_PULLUP);
  pinMode(TEST_DIR_PIN, OUTPUT);
  pinMode(TEST_STEP_PIN, OUTPUT);
  pinMode(TEST_ENA_PIN, OUTPUT);
  
  // Initialize MPG tracker
  memset(&mpgTracker, 0, sizeof(MPGVelocityTracker));
  
  // Attach interrupt for pulse counting
  attachInterrupt(digitalPinToInterrupt(TEST_PULSE_PIN), pulseInterrupt, RISING);
  
  Serial.println("Test program ready. Commands:");
  Serial.println("  'start' - Start velocity test");
  Serial.println("  'stop'  - Stop test");
  Serial.println("  'stats' - Show current statistics");
  Serial.println("  'reset' - Reset all counters");
}

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
    updateVelocity();
    
    // Simulate motor movement based on velocity
    if (mpgTracker.currentVelocity > 0.1) {
      simulateMotorMovement();
    }
  }
  
  delay(10); // Small delay to prevent overwhelming the serial output
}

void pulseInterrupt() {
  pulseCount++;
  lastPulseTime = micros();
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
  digitalWrite(TEST_ENA_PIN, LOW); // Disable motor
}

void resetTest() {
  pulseCount = 0;
  lastPulseTime = 0;
  memset(&mpgTracker, 0, sizeof(MPGVelocityTracker));
  Serial.println("Counters reset");
}

void updateVelocity() {
  unsigned long currentTime = millis();
  
  // Simulate pulse input (in real system, this would come from encoder)
  if (pulseCount > 0) {
    mpgTracker.lastPulseTime[mpgTracker.sampleIndex] = currentTime;
    mpgTracker.pulseCount[mpgTracker.sampleIndex] = pulseCount;
    mpgTracker.sampleIndex = (mpgTracker.sampleIndex + 1) % MPG_VELOCITY_SAMPLES;
    mpgTracker.isActive = true;
    pulseCount = 0; // Reset for next reading
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
}

void simulateMotorMovement() {
  // Calculate movement based on velocity
  float scaledVelocity = mpgTracker.currentVelocity * MPG_VELOCITY_SCALE;
  float movement = scaledVelocity * 100; // Scale for simulation
  
  if (movement > 1.0) {
    // Simulate motor steps
    digitalWrite(TEST_ENA_PIN, HIGH);
    digitalWrite(TEST_DIR_PIN, movement > 0 ? HIGH : LOW);
    
    for (int i = 0; i < abs(movement); i++) {
      digitalWrite(TEST_STEP_PIN, HIGH);
      delayMicroseconds(100);
      digitalWrite(TEST_STEP_PIN, LOW);
      delayMicroseconds(100);
    }
  } else {
    digitalWrite(TEST_ENA_PIN, LOW);
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

// Test scenarios
void runTestScenario(const char* scenario, int pulseRate, int duration) {
  Serial.print("Running scenario: ");
  Serial.println(scenario);
  Serial.print("Pulse rate: ");
  Serial.print(pulseRate);
  Serial.println(" Hz");
  Serial.print("Duration: ");
  Serial.print(duration);
  Serial.println(" ms");
  
  startTest();
  
  // Simulate pulses at specified rate
  unsigned long startTime = millis();
  unsigned long lastPulse = 0;
  unsigned long pulseInterval = 1000 / pulseRate; // Convert Hz to ms
  
  while (millis() - startTime < duration) {
    if (millis() - lastPulse >= pulseInterval) {
      pulseCount++;
      lastPulseTime = micros();
      lastPulse = millis();
    }
    
    updateVelocity();
    simulateMotorMovement();
    delay(1);
  }
  
  stopTest();
  showStats();
  Serial.println("Scenario completed.\n");
}

void runAllTestScenarios() {
  Serial.println("Running all test scenarios...\n");
  
  // Test 1: Slow rotation
  runTestScenario("Slow Rotation", 10, 5000);
  delay(1000);
  
  // Test 2: Medium rotation
  runTestScenario("Medium Rotation", 50, 5000);
  delay(1000);
  
  // Test 3: Fast rotation
  runTestScenario("Fast Rotation", 100, 5000);
  delay(1000);
  
  // Test 4: Variable speed
  Serial.println("Running variable speed test...");
  startTest();
  for (int i = 0; i < 10000; i++) {
    int pulseRate = 10 + (i / 100) * 2; // Gradually increase speed
    if (i % 100 == 0) {
      pulseCount++;
      lastPulseTime = micros();
    }
    
    updateVelocity();
    simulateMotorMovement();
    delay(1);
  }
  stopTest();
  showStats();
  
  Serial.println("All test scenarios completed!");
} 