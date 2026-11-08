// ===== pd_hold.cpp - encoder PD turns + gyro hold (from simple_pd_fire) =====

#include "pd_hold.h"
#include "gyro.h"

// ---------------- pins ----------------
static const int motorPWM = 5;          // driver EN
static const int motorDirection = 6;    // driver PH
static const int sleepPin = 7;          // driver nSLEEP
static const int encoderA = 2;
static const int encoderB = 3;

// ---------------- motor PD ----------------
static const float kp = 1.0;
static const float kd = 0.05;
static const int MAX_PWM = 100;         // top speed of a turn (60 was too weak; up to ~150 if needed)
static const int MIN_PWM = 22;          // smallest push - a bit above the 17 deadband
static const int DONE_TOL = 10;         // counts
static const unsigned long LOOP_US = 5000;

// ---------------- turning and holding ----------------
static const float R_RATIO = 0.14;                            // body angle / wheel angle
static const float COUNTS_PER_DEG = 2800.0 / 360.0 / R_RATIO; // ~55.6 counts per body degree
static const int TURN_SIGN = -1;        // + wheel counts turn the body clockwise
static const float HOLD_TOL = 1.0;      // deg, hold corrects above this
static const unsigned long CHECK_MS = 300;  // wait after a move before checking again
static const float MAX_ANGLE = 360;     // deg either way, so the wire can't wind up
static const bool DEBUG = false;  // prints are commented out: runs hanging with no USB

// ================= state =================
static volatile long pos = 0;
static volatile byte lastAB = 0;
static const int8_t QDEC[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

static long target = 0;                 // wheel target, counts
static long lastPos = 0;
static unsigned long lastLoop = 0;
static unsigned long moveStart = 0;
static unsigned long timeoutMs = 0;
static bool moving = false;

static float holdAngle = 0;             // the body angle to hold, deg
static float countsPerDeg = COUNTS_PER_DEG;  // learnt from the first big turn
static bool learnNext = false;
static float turnStartAngle = 0;
static long turnCounts = 0;
static unsigned long lastMoveEndMs = 0;
static bool still = false;              // on target and not moving
static unsigned long stillSinceMs = 0;

// ================= motor =================
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

static void brake() {
  analogWrite(motorPWM, 0);           // EN low = brake (PH/EN mode)
  digitalWrite(motorDirection, LOW);  // PH low too, in case PMODE comes loose
}

static void drive(int u) {
  if (u == 0) {
    brake();
    return;
  }
  digitalWrite(motorDirection, u > 0 ? HIGH : LOW);
  analogWrite(motorPWM, abs(u));
}

static void moveBy(long counts) {
  long now = getCount();
  target = now + counts;
  lastPos = now;
  moveStart = millis();
  lastLoop = micros();
  timeoutMs = 1000 + labs(counts);    // 1 s + 1 ms per count
  moving = true;
}

static void motorUpdate() {
  if (!moving) return;
  if (micros() - lastLoop < LOOP_US) return;
  lastLoop += LOOP_US;

  long count = getCount();
  long error = target - count;
  float speed = (count - lastPos) / 0.005;
  lastPos = count;

  bool done = labs(error) <= DONE_TOL && fabs(speed) < 200;
  if (done || millis() - moveStart > timeoutMs) {
    brake();
    moving = false;
    lastMoveEndMs = millis();
    return;
  }

  float u = kp * error - kd * speed;  // the PD
  if (u > 0 && u < MIN_PWM) u = MIN_PWM;
  if (u < 0 && u > -MIN_PWM) u = -MIN_PWM;
  u = constrain(u, -MAX_PWM, MAX_PWM);
  drive((int)u);
}

// ================= turn + hold =================
static void holdUpdate() {
  if (moving) return;

  // a big turn just finished: learn how many counts one body degree really is
  if (learnNext) {
    learnNext = false;
    float turned = gyroAngle() - turnStartAngle;
    float commanded = TURN_SIGN * turnCounts / countsPerDeg;
    if (turned * commanded < 0) {
      // Serial.println(F("WARNING: gyro turned the other way - set GYRO_SIGN = -1"));
    } else if (fabs(turned) > 10) {
      float cpd = fabs(turnCounts / turned);
      if (cpd > COUNTS_PER_DEG / 3 && cpd < COUNTS_PER_DEG * 3) countsPerDeg = cpd;
    }
  }

  if (millis() - lastMoveEndMs < CHECK_MS) return;   // let it settle after a move
  lastMoveEndMs = millis();
  float err = holdAngle - gyroAngle();
  if (fabs(err) > HOLD_TOL) {
    moveBy(lround(TURN_SIGN * err * countsPerDeg));  // small move back onto target
    still = false;
  } else if (!still) {
    still = true;
    stillSinceMs = millis();
  }
}

// ================= public =================
void pdBegin() {
  pinMode(sleepPin, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  brake();
  digitalWrite(sleepPin, HIGH);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  lastAB = digitalRead(encoderA) << 1 | digitalRead(encoderB);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderISR, CHANGE);
}

void pdHoldHere() {
  holdAngle = gyroAngle();
  still = false;
  lastMoveEndMs = millis();
}

// Stop any move now and hold the angle the body is at.
void pdStopHere() {
  brake();
  moving = false;
  learnNext = false;                   // a cut-short turn can't teach the counts per degree
  holdAngle = gyroAngle();
  still = false;
  lastMoveEndMs = millis();
}

// turn the body by deg (+ = anticlockwise / left), then hold there
void pdTurnBy(float deg) {
  float end = holdAngle + deg;
  if (end > MAX_ANGLE) deg -= 360;   // same heading, other way round
  if (end < -MAX_ANGLE) deg += 360;
  holdAngle += deg;
  turnStartAngle = gyroAngle();
  turnCounts = lround(TURN_SIGN * deg * countsPerDeg);
  learnNext = fabs(deg) >= 30;
  moveBy(turnCounts);
  still = false;
  // if (DEBUG) {
  //   Serial.print(F("turn "));
  //   Serial.print(deg, 0);
  //   Serial.print(F(" -> hold "));
  //   Serial.println(holdAngle, 1);
  // }
}

void pdUpdate() {
  motorUpdate();
  holdUpdate();
}

bool pdStill() {
  return still;
}

unsigned long pdStillSinceMs() {
  return stillSinceMs;
}

float pdHoldAngle() {
  return holdAngle;
}
