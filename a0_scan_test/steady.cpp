// ===== steady.cpp - gyro turn + drift hold =====
// First turn from rest: turnBy on the encoder, brake, let it drift 1 s while
// the gyro measures the drift rate, spin the wheel at the steady speed that
// cancels it, turn back onto the target -> READY.
// While READY: every 0.5 s re-measure the drift that is left and adjust the
// wheel speed; more than 0.3 deg off the target -> turn back.
// Later turns run on top of the spinning wheel - it is never braked, because
// braking hands the wheel's speed back to the body (jerk, then drift).
// How far the body really turns per commanded degree (k) is learnt from the
// first big turn, so it still works when r is not 0.14 (e.g. lighter skeleton).

#include "steady.h"
#include "motor.h"
#include "gyro.h"

static const unsigned long SETTLE_MS = 300;   // after the brake
static const unsigned long MEASURE_MS = 1000; // let it drift, gyro measures
static const unsigned long RETURN_MS = 1500;  // max time to get back after measuring
static const unsigned long RECHECK_MS = 500;  // re-measure the drift this often
static const float READY_TOL = 0.5;           // deg, READY inside this
static const float DRIFT_TOL = 0.3;           // deg, turn back above this while holding

static SteadyState state = STEADY_OFF;
static unsigned long stateStart = 0;
static unsigned long waitMs = 0;              // RETURN time limit
static float holdAngle = 0;
static float drift = 0;
static float correction = 0;
static float k = 1;                           // real body deg per commanded deg
static float wheelCmd = 0;                    // hold speed now, commanded deg/s
static float turnStart = 0;
static float turnDeg = 0;
static float measureStart = 0;
static unsigned long lastCheck = 0;
static float lastAngle = 0;
static float nudgedDeg = 0;                   // turned back on purpose last window

static void enter(SteadyState s) {
  state = s;
  stateStart = millis();
}

// turn by deg on top of the running wheel speed (no brake)
static void turnInHold(float deg, unsigned long ms) {
  motorHoldNudge(deg / k);
  waitMs = ms;
  enter(STEADY_RETURN);
}

static void startChecks() {
  lastCheck = millis();
  lastAngle = gyroAngle();
  nudgedDeg = 0;
}

void steadyTurnBy(float deg) {
  holdAngle = gyroAngle() + deg;         // the target, on the gyro
  correction = 0;
  if (motorHolding()) {
    turnInHold(deg, 1000 + (unsigned long)(fabs(deg) * 60));   // 90 deg -> 6.4 s
  } else {
    drift = 0;
    turnStart = gyroAngle();
    turnDeg = deg;
    turnBy(deg);
    enter(STEADY_TURNING);
  }
}

void steadyStart() {
  steadyStartAt(gyroAngle());
}

void steadyStartAt(float angle) {
  holdAngle = angle;
  correction = 0;
  if (motorHolding()) {
    turnInHold(holdAngle - gyroAngle(), RETURN_MS);
  } else {
    drift = 0;
    enter(STEADY_SETTLE);
  }
}

void steadyUpdate() {
  unsigned long t = millis() - stateStart;

  switch (state) {
    case STEADY_TURNING: {
      // encoder turn from rest; when it ends (DONE or TIMEOUT) measure
      if (motorBusy()) break;
      if (motorResult() == MOTOR_STOPPED) {
        state = STEADY_OFF;
        break;
      }
      float turned = gyroAngle() - turnStart;
      if (fabs(turnDeg) >= 30) {
        float kk = turned / turnDeg;
        if (kk > 0.3 && kk < 3) k = kk;  // ignore nonsense (gyro not reading)
      }
      enter(STEADY_SETTLE);
      break;
    }

    case STEADY_SETTLE:
      if (t >= SETTLE_MS) {
        measureStart = gyroAngle();
        enter(STEADY_MEASURE);
      }
      break;

    case STEADY_MEASURE:
      // wheel braked: body + wheel drift together, gyro measures it
      if (t >= MEASURE_MS) {
        drift = (gyroAngle() - measureStart) / (MEASURE_MS / 1000.0);
        wheelCmd = -drift / k;
        motorHold(wheelCmd);             // steady wheel speed that cancels it
        correction = holdAngle - gyroAngle();
        turnInHold(correction, RETURN_MS);   // back onto the target
      }
      break;

    case STEADY_RETURN:
      if (fabs(holdAngle - gyroAngle()) < READY_TOL || t >= waitMs) {
        startChecks();
        enter(STEADY_READY);
      }
      break;

    case STEADY_READY: {
      // every RECHECK_MS: add the drift that is left to the wheel speed,
      // turn back if more than DRIFT_TOL off
      if (millis() - lastCheck < RECHECK_MS) break;
      float secs = (millis() - lastCheck) / 1000.0;
      lastCheck = millis();
      float leftDrift = (gyroAngle() - lastAngle - nudgedDeg) / secs;
      wheelCmd -= leftDrift / k;
      motorHoldSpeed(wheelCmd);
      nudgedDeg = 0;
      float err = holdAngle - gyroAngle();
      if (fabs(err) > DRIFT_TOL) {
        motorHoldNudge(err / k);
        nudgedDeg = err;
      }
      lastAngle = gyroAngle();
      break;
    }

    default:
      break;
  }
}

void steadyStop() {
  // Only clears the steady state. The wheel keeps its hold speed so the next
  // turn runs on top of it (braking would hand the drift back to the body).
  // Call motorStop() as well to brake.
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
