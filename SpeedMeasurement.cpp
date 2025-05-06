#include <Arduino.h>

// Pin definition
const int signalPin1 = 27;
const int signalPin2 = 25;
const int signalPin3 = 26;

// Variables for measuring state duration
volatile uint32_t stateStartTime1 = 0;   // Start time of the last state for Signal 1
volatile uint32_t highDuration1 = 0;     // Duration of HIGH state for Signal 1
volatile uint32_t lowDuration1 = 0;      // Duration of LOW state for Signal 1
volatile bool newCycleDurationAvailable1 = false;  // Flag indicating a new cycle duration is available for Signal 1

volatile uint32_t stateStartTime2 = 0;   // Start time of the last state for Signal 2
volatile uint32_t highDuration2 = 0;     // Duration of HIGH state for Signal 2
volatile uint32_t lowDuration2 = 0;      // Duration of LOW state for Signal 2
volatile bool newCycleDurationAvailable2 = false;  // Flag indicating a new cycle duration is available for Signal 2

volatile uint32_t stateStartTime3 = 0;   // Start time of the last state for Signal 3
volatile uint32_t highDuration3 = 0;     // Duration of HIGH state for Signal 3
volatile uint32_t lowDuration3 = 0;      // Duration of LOW state for Signal 3
volatile bool newCycleDurationAvailable3 = false;  // Flag indicating a new cycle duration is available for Signal 3

volatile bool outputFlag = false;  // Flag indicating whether to output the RPM values

volatile int sensorState1 = 0;     // Current state of Signal 1
volatile int sensorState2 = 0;     // Current state of Signal 2
volatile int sensorState3 = 0;     // Current state of Signal 3

float rpm1 = 0.0;      // RPM value for Signal 1
float rpm2 = 0.0;      // RPM value for Signal 2
float rpm3 = 0.0;      // RPM value for Signal 3
float latestRpm = 0.0; // Most recent RPM value
float averageRpm = 0.0; // Average RPM value

enum Direction {
  UNKNOWN = 3,
  CLOCKWISE = 1,
  COUNTERCLOCKWISE = 2
};

volatile Direction direction = UNKNOWN;  // Current rotation direction
volatile int previousState = 0;   // Previous combined state of the sensors
volatile int currentState = 0;    // Current combined state of the sensors
volatile int lastDirection = UNKNOWN;  // Last detected rotation direction
volatile int directionCount = 0;  // Counter for confirmed direction changes
volatile uint32_t lastChangeTime = 0;  // Time of the last signal level change

// Function to update the rotation direction based on current sensor states
void IRAM_ATTR updateDirection() {
  // Combine the states of the three sensors into a single current state
  currentState = (sensorState1 << 2) | (sensorState2 << 1) | sensorState3;

  // Check transitions between states to determine the rotation direction
  int newDirection = UNKNOWN;
  if ((previousState == 0b001 && currentState == 0b011) ||
      (previousState == 0b011 && currentState == 0b010) ||
      (previousState == 0b010 && currentState == 0b110) ||
      (previousState == 0b110 && currentState == 0b100) ||
      (previousState == 0b100 && currentState == 0b101) ||
      (previousState == 0b101 && currentState == 0b001)) {
    newDirection = CLOCKWISE;
  } else if ((previousState == 0b001 && currentState == 0b101) ||
             (previousState == 0b101 && currentState == 0b100) ||
             (previousState == 0b100 && currentState == 0b110) ||
             (previousState == 0b110 && currentState == 0b010) ||
             (previousState == 0b010 && currentState == 0b011) ||
             (previousState == 0b011 && currentState == 0b001)) {
    newDirection = COUNTERCLOCKWISE;
  }

  // If the new direction is recognized and differs from the last recognized direction
  if (newDirection != UNKNOWN && newDirection != lastDirection) {
    lastDirection = newDirection;
    directionCount = 1;  // Start counting to confirm the direction change
  } else if (newDirection == lastDirection) {
    directionCount++;  // Increase the counter for the same direction
    if (directionCount >= 3) {
      direction = (Direction)newDirection;  // Confirm the direction change
      directionCount = 0;  // Reset the counter to prevent overflow
    }
  }

  // Update the last known state
  previousState = currentState;
  lastChangeTime = millis();  // Update the time of the last change
}

// ISR for interrupt on Pin 27
void IRAM_ATTR handleInterrupt1() {
  uint32_t currentTime = micros();  // Read the current time in microseconds
  int pinState = digitalRead(signalPin1);  // Read the current state of the pin

  // Calculate the duration of HIGH or LOW state
  if (pinState == HIGH) {
    lowDuration1 = currentTime - stateStartTime1;
  } else {
    highDuration1 = currentTime - stateStartTime1;
    newCycleDurationAvailable1 = true;  // Mark new cycle duration as available
  }

  // Update start time and sensor state
  stateStartTime1 = currentTime;
  sensorState1 = pinState;

  // Update the rotation direction based on current states
  updateDirection();
}

// ISR for interrupt on Pin 25
void IRAM_ATTR handleInterrupt2() {
  uint32_t currentTime = micros();  // Read the current time in microseconds
  int pinState = digitalRead(signalPin2);  // Read the current state of the pin

  // Calculate the duration of HIGH or LOW state
  if (pinState == HIGH) {
    lowDuration2 = currentTime - stateStartTime2;
  } else {
    highDuration2 = currentTime - stateStartTime2;
    newCycleDurationAvailable2 = true;  // Mark new cycle duration as available
  }

  // Update start time and sensor state
  stateStartTime2 = currentTime;
  sensorState2 = pinState;

  // Update the rotation direction based on current states
  updateDirection();
}

// ISR for interrupt on Pin 26
void IRAM_ATTR handleInterrupt3() {
  uint32_t currentTime = micros();  // Read the current time in microseconds
  int pinState = digitalRead(signalPin3);  // Read the current state of the pin

  // Calculate the duration of HIGH or LOW state
  if (pinState == HIGH) {
    lowDuration3 = currentTime - stateStartTime3;
  } else {
    highDuration3 = currentTime - stateStartTime3;
    newCycleDurationAvailable3 = true;  // Mark new cycle duration as available
  }

  // Update start time and sensor state
  stateStartTime3 = currentTime;
  sensorState3 = pinState;

  // Update the rotation direction based on current states
  updateDirection();
}

// Timer callback for serial output
void IRAM_ATTR onTimer() {
  outputFlag = true;  // Set the flag to trigger RPM output
}

void setup() {
  // Initialize serial communication for output
  Serial.begin(921600);

  // Configure Hall sensor pins as inputs with pull-up resistors
  pinMode(signalPin1, INPUT_PULLUP);
  pinMode(signalPin2, INPUT_PULLUP);
  pinMode(signalPin3, INPUT_PULLUP);

  // Attach interrupts for the Hall sensors
  attachInterrupt(digitalPinToInterrupt(signalPin1), handleInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(signalPin2), handleInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(signalPin3), handleInterrupt3, CHANGE);

  // Configure a hardware timer for periodic output
  hw_timer_t * timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1 µs tick)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);  // 100000 µs = 100 ms
  timerAlarmEnable(timer);

  lastChangeTime = millis();  // Initialize the time of the last change
}

void loop() {
  // Check if the timer requested output
  if (outputFlag) {
    noInterrupts();  // Disable interrupts to safely access shared resources

    // Check if new cycle durations are available
    bool ready1 = newCycleDurationAvailable1;
    bool ready2 = newCycleDurationAvailable2;
    bool ready3 = newCycleDurationAvailable3;

    // Reset the flags
    newCycleDurationAvailable1 = false;
    newCycleDurationAvailable2 = false;
    newCycleDurationAvailable3 = false;
    outputFlag = false;

    interrupts();  // Re-enable interrupts

    // Check if a change occurred within the last 1000 ms
    if (millis() - lastChangeTime >= 1000) {
      // Set all speeds to 0 if no signal change occurred in 1 second
      rpm1 = 0.0;
      rpm2 = 0.0;
      rpm3 = 0.0;
      latestRpm = 0.0;
      averageRpm = 0.0;
    } else {
      // Calculate RPM for the three sensors if new data is available
      if (ready1) {
        float cycleDuration1 = highDuration1 + lowDuration1;
        rpm1 = (60.0 * 1000000.0) / (cycleDuration1 * 2);
        latestRpm = rpm1;
      }
      if (ready2) {
        float cycleDuration2 = highDuration2 + lowDuration2;
        rpm2 = (60.0 * 1000000.0) / (cycleDuration2 * 2);
        latestRpm = rpm2;
      }
      if (ready3) {
        float cycleDuration3 = highDuration3 + lowDuration3;
        rpm3 = (60.0 * 1000000.0) / (cycleDuration3 * 2);
        latestRpm = rpm3;
      }

      // Calculate the average RPM
      averageRpm = -(rpm1 + rpm2 + rpm3) / 3.0;

      // Make average RPM positive for CW and negative for CCW
      if (direction == COUNTERCLOCKWISE) {
        averageRpm = -averageRpm;
      }
    }

    // Output the RPM values via the serial interface
    Serial.print(rpm1, 2);
    Serial.print(",");
    Serial.print(rpm2, 2);
    Serial.print(",");
    Serial.print(rpm3, 2);
    Serial.print(",");
    Serial.print(latestRpm, 2);
    Serial.print(",");
    Serial.println(averageRpm, 2);  // Average RPM, positive or negative depending on direction
  }
}
