#include "Arduino.h"
#include "motor_control.h"

const int motorPWM = 5;
const int motorDirection = 6;
const int encoderA = 2;
const int encoderB = 3;
const int sleep = 7;
const int turnSpeed = 150;
float countsPerDegree = 700.0 / 360.0;
volatile long pos = 0;   // long: an int overflows after ~46 revs of continuous scanning

// ---- closed-loop position control (PD) ----
// Tune on the real turret: raise KP until it reaches the target briskly,
// then raise KD until the overshoot/oscillation goes away.
const float KP = 3.0;              // PWM per count of error
const float KD = 20.0;             // PWM per (count/ms) of speed, damps overshoot
const int   MIN_PWM = 60;          // below this the motor won't overcome friction
const int   POS_TOLERANCE = 2;     // counts (~1 deg) counted as "there"
const int   SETTLE_SAMPLES = 5;    // must stay in tolerance this many periods
const unsigned long CONTROL_PERIOD_US = 2000;
// If motorForward() makes pos go DOWN, the loop runs away: set this to -1.
const int   ENCODER_SIGN = 1;

static long targetPos = 0;         // absolute target, so step errors don't accumulate
static bool atTarget = true;

void readEncoder() {
  int b = digitalRead(encoderB);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}

long readPos() {
  noInterrupts();                  // a long is 4 bytes; read it atomically
  long p = pos;
  interrupts();
  return p * ENCODER_SIGN;
}

void motorForward(int speedValue) {
  digitalWrite(motorDirection, HIGH);
  analogWrite(motorPWM, speedValue);
}

void motorReverse(int speedValue) {
  digitalWrite(motorDirection, LOW);
  analogWrite(motorPWM, speedValue);
}

void motorStop() {
  analogWrite(motorPWM, 0);
}

// Drive to an absolute encoder count. Blocks until settled or timed out.
bool moveTo(long target, int maxSpeed, unsigned long timeoutMs) {
  targetPos = target;
  atTarget = false;

  long prevPos = readPos();
  int settled = 0;
  unsigned long startMs = millis();
  unsigned long lastUs = micros();

  while (millis() - startMs < timeoutMs) {
    if (micros() - lastUs < CONTROL_PERIOD_US) continue;
    float dtMs = (micros() - lastUs) / 1000.0;
    lastUs = micros();

    long p = readPos();
    long err = target - p;
    float speed = (p - prevPos) / dtMs;      // counts per ms
    prevPos = p;

    if (abs(err) <= POS_TOLERANCE) {
      motorStop();
      if (++settled >= SETTLE_SAMPLES) {
        atTarget = true;
        return true;
      }
      continue;
    }
    settled = 0;

    float u = KP * err - KD * speed;
    int pwm = constrain((int)abs(u), MIN_PWM, maxSpeed);
    if (u > 0) motorForward(pwm);
    else       motorReverse(pwm);
  }

  motorStop();                               // safety timeout
  return false;
}

// Move relative to the last target (not the current position).
bool stepDegrees(float angle, int maxSpeed) {
  return moveTo(targetPos + lround(angle * countsPerDegree), maxSpeed, 2000);
}

void turnAngle(float angle, int speedValue) {
  stepDegrees(angle, speedValue);
}

void rotateTo(float angle) {
  turnAngle(angle, turnSpeed);
}

bool isAtTarget() {
  return atTarget;
}

void setupMotor() {
  pinMode(sleep, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(sleep, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);
}
