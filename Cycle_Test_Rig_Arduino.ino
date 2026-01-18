// Fixed for Arduino UNO (AVR):
// - Removed IRAM_ATTR (ESP-only attribute causing compile error on Uno)
// - Added <math.h> for lroundf
// - Everything else unchanged from the 3000pps version

#include <AccelStepper.h>
#include <math.h>

#define STEP_PIN    5
#define DIR_PIN     4
#define ENABLE_PIN  6
#define BUTTON_PIN  2

const float linearPerStep      = 0.0127f;
const float knobMovementUp     = 14.0f;
const float knobMovementDown   = 14.0f;
const float counterMovementDown= 12.0f;
const float counterMovementUp  = 12.0f;

const long  maxPPS   = 7000;      // Target speed: 3000 pulses/sec
const long  accelPPS2= 100000L;     // Acceleration: 30000 steps/s^2

const unsigned long time1 = 10000UL;
const unsigned long time2 = 10000UL;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

enum State {
  IDLE,
  KNOB_UP_1,
  KNOB_DOWN_1,
  WAIT1,
  KNOB_UP_2,
  KNOB_DOWN_2,
  COUNTER_DOWN,
  COUNTER_UP,
  WAIT2,
  RETURN_TO_ZERO
};

State currentState = IDLE;

volatile bool buttonISRFlag = false;
bool cycleRunning = false;
unsigned long stateStartTime = 0;
unsigned long lastButtonHandledMs = 0;
const unsigned long debounceMs = 120;

inline long mmToSteps(float mm) {
  return (long)lroundf(mm / linearPerStep);
}

void moveToMM(float mm) {
  long targetSteps = mmToSteps(mm);
  if (stepper.targetPosition() != targetSteps) {
    stepper.moveTo(targetSteps);
  }
}

// ISR for UNO (no attributes needed)
void toggleISR() {
  buttonISRFlag = true;
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(ENABLE_PIN, LOW);

  stepper.setMaxSpeed(maxPPS);
  stepper.setAcceleration(accelPPS2);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggleISR, FALLING);
}

void loop() {
  stepper.run();

  if (buttonISRFlag) {
    unsigned long now = millis();
    if (now - lastButtonHandledMs >= debounceMs) {
      lastButtonHandledMs = now;
      buttonISRFlag = false;

      cycleRunning = !cycleRunning;
      if (cycleRunning && currentState == IDLE) {
        currentState = KNOB_UP_1;
      }
    } else {
      buttonISRFlag = false;
    }
  }

  if (!cycleRunning && currentState != IDLE && currentState != RETURN_TO_ZERO) {
    currentState = RETURN_TO_ZERO;
  }

  switch (currentState) {
    case IDLE: break;
    case KNOB_UP_1:
      moveToMM(knobMovementUp);
      if (stepper.distanceToGo() == 0) currentState = KNOB_DOWN_1;
      break;
    case KNOB_DOWN_1:
      moveToMM(0);
      if (stepper.distanceToGo() == 0) {
        currentState = WAIT1;
        stateStartTime = millis();
      }
      break;
    case WAIT1:
      if (millis() - stateStartTime >= time1) currentState = KNOB_UP_2;
      break;
    case KNOB_UP_2:
      moveToMM(knobMovementUp);
      if (stepper.distanceToGo() == 0) currentState = KNOB_DOWN_2;
      break;
    case KNOB_DOWN_2:
      moveToMM(0);
      if (stepper.distanceToGo() == 0) currentState = COUNTER_DOWN;
      break;
    case COUNTER_DOWN:
      moveToMM(-counterMovementDown);
      if (stepper.distanceToGo() == 0) currentState = COUNTER_UP;
      break;
    case COUNTER_UP:
      moveToMM(0);
      if (stepper.distanceToGo() == 0) {
        currentState = WAIT2;
        stateStartTime = millis();
      }
      break;
    case WAIT2:
      if (millis() - stateStartTime >= time2) {
        if (cycleRunning) currentState = KNOB_UP_1;
        else currentState = RETURN_TO_ZERO;
      }
      break;
    case RETURN_TO_ZERO:
      moveToMM(0);
      if (stepper.distanceToGo() == 0) currentState = IDLE;
      break;
  }
}
