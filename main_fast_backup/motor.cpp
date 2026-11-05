// ===== motor.cpp - reaction wheel PD (from pd_controller) =====
// Everything here is static (private to this file) except the functions in
// motor.h, so nothing clashes with the sensor or logic code.

#include "motor.h"

// ----- pins -----
static const int motorPWM = 5;          // driver EN
static const int motorDirection = 6;    // driver PH
static const int sleepPin = 7;          // driver nSLEEP
static const int encoderA = 2;
static const int encoderB = 3;

// ----- calibration -----
// r = body angle / wheel angle. 0.14 checked on the hanging satellite
// (4 x 90 deg turns ended close to the start, 2026-10-23).
static const float R_RATIO = 0.14;
static const float COUNTS_PER_REV = 2800.0;            // 700 CPR x 4
static const float COUNTS_PER_BODY_DEG = COUNTS_PER_REV / 360.0 / R_RATIO;   // ~55.6
// A + wheel move turns the body CLOCKWISE (seen from above, hang test).
// turnBy(+) should be anticlockwise, so flip the sign. If turnBy(+) turns
// the body clockwise on your setup, change this to +1.
static const int TURN_SIGN = -1;

// ----- PD constants -----
static const float kp = 1.0;
static const float kd = 0.05;
static const int MAX_PWM = 100;
static const int MIN_PWM = 17;          // motor deadband
static const int DONE_TOL = 10;         // counts (~0.2 deg of body)
static const unsigned long LOOP_US = 5000;

// ----- encoder -----
static volatile long pos = 0;
static volatile byte lastAB = 0;
static const int8_t QDEC[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// ----- PD state -----
static long target = 0;
static long lastPos = 0;
static unsigned long lastLoop = 0;
static unsigned long moveStart = 0;
static unsigned long timeoutMs = 0;
static bool moving = false;
static int lastPwm = 0;
static MotorResult result = MOTOR_IDLE;

// ----- hold state (wheel keeps turning at a steady speed after a turn) -----
static bool holding = false;
static float holdSpeed = 0;             // wheel counts per second
static float holdStartCount = 0;        // float so speed changes don't lose counts
static unsigned long holdStartMs = 0;

// ----- low level -----
static void encoderISR() {
  int a = digitalRead(encoderA);
  int b = digitalRead(encoderB);
  int ab = a << 1 | b;
  pos += QDEC[(lastAB << 2) | ab];
  lastAB = ab;
}

static long getCount() {
  noInterrupts();
  long c = pos;
  interrupts();
  return c;
}

static void motorForward(int speedValue) {
  digitalWrite(motorDirection, HIGH);
  analogWrite(motorPWM, speedValue);
}

static void motorReverse(int speedValue) {
  digitalWrite(motorDirection, LOW);
  analogWrite(motorPWM, speedValue);
}

static void brake() {
  analogWrite(motorPWM, 0);             // EN low = brake in PH/EN mode
}

static void drive(int u) {
  if (u > 0) {
    motorForward(u);
  } else if (u < 0) {
    motorReverse(-u);
  } else {
    brake();
  }
}

static void finish(MotorResult r) {
  drive(0);
  lastPwm = 0;
  moving = false;
  holding = false;
  result = r;
}

// ----- public -----
void motorBegin() {
  pinMode(sleepPin, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  brake();
  digitalWrite(sleepPin, HIGH);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  int a = digitalRead(encoderA);
  int b = digitalRead(encoderB);
  lastAB = a << 1 | b;
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderISR, CHANGE);
}

void moveWheelBy(long counts) {
  long now = getCount();
  target = now + counts;
  lastPos = now;
  moveStart = millis();
  lastLoop = micros();
  timeoutMs = 1000 + labs(counts) / 2;  // 1 s + 2 ms per 4 counts (90 deg -> 3.5 s)
  moving = true;
  holding = false;
  result = MOTOR_BUSY;
}

void turnBy(float bodyDeg) {
  long counts = lround(TURN_SIGN * bodyDeg * COUNTS_PER_BODY_DEG);
  moveWheelBy(counts);
}

static float toHoldSpeed(float bodyDegPerSec) {
  float s = TURN_SIGN * bodyDegPerSec * COUNTS_PER_BODY_DEG;
  if (s > 3000) s = 3000;                     // about full speed at MAX_PWM
  if (s < -3000) s = -3000;
  return s;
}

void motorHold(float bodyDegPerSec) {
  // Same sign as turnBy: the wheel speed that would turn the body at
  // bodyDegPerSec on its own. To cancel a drift of +d deg/s, use -d.
  holdSpeed = toHoldSpeed(bodyDegPerSec);
  long now = getCount();
  holdStartCount = now;
  holdStartMs = millis();
  target = now;
  lastPos = now;
  lastLoop = micros();
  moving = false;
  holding = true;
  result = MOTOR_HOLDING;
}

void motorHoldNudge(float bodyDeg) {
  // While holding: turn the body by bodyDeg on top of the steady speed
  // (shifts the moving target, the PD follows). Same sign as turnBy.
  if (!holding) return;
  holdStartCount += TURN_SIGN * bodyDeg * COUNTS_PER_BODY_DEG;
}

void motorHoldSpeed(float bodyDegPerSec) {
  // While holding: change the steady speed without a jump in the target
  // (used by steady's continuous hold, every 20 ms).
  if (!holding) {
    motorHold(bodyDegPerSec);
    return;
  }
  unsigned long now = millis();
  holdStartCount += holdSpeed * (now - holdStartMs) / 1000.0;
  holdStartMs = now;
  holdSpeed = toHoldSpeed(bodyDegPerSec);
}

void motorUpdate() {
  // A. nothing to do if no move or hold is running
  if (!moving && !holding) return;

  // B. only run every 5 ms; dt measured so a late call (e.g. after a
  //    sensor read during a hold) doesn't upset the speed estimate
  unsigned long nowUs = micros();
  if (nowUs - lastLoop < LOOP_US) return;
  float dt = (nowUs - lastLoop) / 1000000.0;
  lastLoop = nowUs;

  // C. measure: position, error, speed
  //    while holding, the target moves at holdSpeed (a ramp the PD follows)
  long count = getCount();
  float refSpeed = 0;
  if (holding) {
    refSpeed = holdSpeed;
    target = lround(holdStartCount + holdSpeed * (millis() - holdStartMs) / 1000.0);
  }
  long error = target - count;              // how far still to go
  float speed = (count - lastPos) / dt;     // counts per second
  lastPos = count;

  if (moving) {
    // D. done? close enough and nearly stopped -> brake and finish
    if (labs(error) <= DONE_TOL && fabs(speed) < 200) {
      finish(MOTOR_DONE);
      return;
    }

    // E. safety: taking too long -> brake and give up
    if (millis() - moveStart > timeoutMs) {
      finish(MOTOR_TIMEOUT);
      return;
    }
  }

  // F. correct: the PD law, then deadband and clamp
  float u = kp * error + kd * (refSpeed - speed);

  if (u > 0 && u < MIN_PWM) u = MIN_PWM;
  if (u < 0 && u > -MIN_PWM) u = -MIN_PWM;

  if (u > MAX_PWM) u = MAX_PWM;
  if (u < -MAX_PWM) u = -MAX_PWM;

  lastPwm = (int)u;
  drive(lastPwm);
}

bool motorBusy() {
  return moving;
}

bool motorHolding() {
  return holding;
}

MotorResult motorResult() {
  return result;
}

void motorStop() {
  finish(MOTOR_STOPPED);
}

long motorCount() {
  return getCount();
}

long motorTarget() {
  return target;
}

int motorPwm() {
  return lastPwm;
}
