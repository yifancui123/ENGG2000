// ===== steady.cpp - gyro turn + measure-once drift cancel =====

#include "steady.h"
#include "motor.h"
#include "gyro.h"

static const unsigned long SETTLE_MS = 100;   // brake transient
static const unsigned long MEASURE_MS = 300;  // averaging window
static const unsigned long CANCEL_MS = 150;   // wheel speeds up, body stops
static const unsigned long RETURN_MS = 1500;  // max time to reach the target
static const float RETURN_TOL = 0.5;          // deg, close enough

// CONTINUOUS = true: after CANCEL, keep correcting with the gyro every
// 20 ms for the whole hold, so a wire that twists back (spring) is fought
// all the time. false = the measure-once hold (one speed + one nudge).
static const bool CONTINUOUS = true;
static const unsigned long LOOP_MS = 20;
static const float ANGLE_GAIN = 3.0;    // deg/s back towards target per deg off
static const float MAX_RETURN = 20.0;   // deg/s, fastest it turns back
static const float RATE_GAIN = 0.15;    // share of the rate error fixed every 20 ms
static const float MAX_WHEEL = 50.0;    // deg/s, wheel limit (motor clamps ~54)

static SteadyState state = STEADY_OFF;
static unsigned long stateStart = 0;
static float holdAngle = 0;
static float drift = 0;
static float correction = 0;
static float rateSum = 0;
static long rateN = 0;
static float wheelCmd = 0;              // hold speed now, body deg/s
static unsigned long lastLoopMs = 0;

static void enter(SteadyState s) {
  state = s;
  stateStart = millis();
}

// continuous hold: body rate should be ANGLE_GAIN x angle error (back to the
// target); the wheel speed is nudged towards that every 20 ms
static void holdLoop() {
  if (millis() - lastLoopMs < LOOP_MS) return;
  lastLoopMs = millis();
  float want = constrain(ANGLE_GAIN * (holdAngle - gyroAngle()), -MAX_RETURN, MAX_RETURN);
  wheelCmd += RATE_GAIN * (want - gyroRate());
  wheelCmd = constrain(wheelCmd, -MAX_WHEEL, MAX_WHEEL);
  motorHoldSpeed(wheelCmd);
}

static void resetMeasure() {
  drift = 0;
  correction = 0;
  rateSum = 0;
  rateN = 0;
}

void steadyTurnBy(float deg) {
  holdAngle = gyroAngle() + deg;         // the target, on the gyro
  resetMeasure();
  turnBy(deg);
  enter(STEADY_TURNING);
}

void steadyStart() {
  steadyStartAt(gyroAngle());
}

void steadyStartAt(float angle) {
  holdAngle = angle;
  resetMeasure();
  enter(STEADY_SETTLE);
}

void steadyUpdate() {
  unsigned long t = millis() - stateStart;

  switch (state) {
    case STEADY_TURNING:
      // encoder turn running; when it ends (DONE or TIMEOUT) hold the target
      if (motorBusy()) break;
      if (motorResult() == MOTOR_STOPPED) {
        state = STEADY_OFF;
        break;
      }
      enter(STEADY_SETTLE);
      break;

    case STEADY_SETTLE:
      if (t >= SETTLE_MS) enter(STEADY_MEASURE);
      break;

    case STEADY_MEASURE:
      // wheel braked: body + wheel drift together, gyro measures it
      rateSum += gyroRate();
      rateN++;
      if (t >= MEASURE_MS) {
        drift = rateN > 0 ? rateSum / rateN : 0;
        wheelCmd = -drift;
        motorHold(wheelCmd);             // steady wheel speed that cancels it
        enter(STEADY_CANCEL);
      }
      break;

    case STEADY_CANCEL:
      // give the wheel a moment to reach the speed, then go to the target
      if (t >= CANCEL_MS) {
        correction = holdAngle - gyroAngle();
        if (!CONTINUOUS) motorHoldNudge(correction);   // else holdLoop returns it
        lastLoopMs = millis();
        enter(STEADY_RETURN);
      }
      break;

    case STEADY_RETURN:
      if (CONTINUOUS) holdLoop();
      if (fabs(holdAngle - gyroAngle()) < RETURN_TOL || t >= RETURN_MS) {
        enter(STEADY_READY);
      }
      break;

    case STEADY_READY:
      if (CONTINUOUS) holdLoop();        // keep fighting the wire
      break;

    default:
      break;
  }
}

void steadyStop() {
  // Only clears the steady state. The wheel keeps its hold speed until the
  // next turn takes over (braking here would hand the drift straight back
  // to the body). Call motorStop() as well to brake.
  state = STEADY_OFF;
}

bool steadyActive() {
  return state != STEADY_OFF;
}

bool steadyReady() {
  return state == STEADY_READY;
}

SteadyState steadyState() {
  return state;
}

float steadyHoldAngle() {
  return holdAngle;
}

float steadyDrift() {
  return drift;
}

float steadyCorrection() {
  return correction;
}
