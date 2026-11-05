// ===== steady.cpp - gyro turn + hold on target (pulse / continuous / once) =====

#include "steady.h"
#include "motor.h"
#include "gyro.h"

static const unsigned long SETTLE_MS = 100;   // brake transient
static const unsigned long MEASURE_MS = 300;  // averaging window
static const unsigned long CANCEL_MS = 150;   // wheel speeds up, body stops
static const unsigned long RETURN_MS = 1500;  // max time to reach the target
static const float RETURN_TOL = 0.5;          // deg, close enough

// How the hold works after the turn:
//   HOLD_PULSE      if the body drifts more than PULSE_DEAD off target, a
//                   small encoder turn back (turnBy) + brake, wait, check
//                   again. Still between pulses. (default)
//   HOLD_CONTINUOUS measure the drift (coarse), then a cascade every 10 ms
//                   (fine): angle PD -> wanted body rate -> rate PD that
//                   adds to the wheel speed. Gyro rate averaged over 5
//                   samples. Fights a wire that twists back.
//   HOLD_ONCE       measure the drift, one cancelling speed + one nudge
enum HoldMode { HOLD_PULSE, HOLD_CONTINUOUS, HOLD_ONCE };
static const HoldMode HOLD_MODE = HOLD_PULSE;

static const float PULSE_DEAD = 0.5;          // deg, no pulse inside this
static const unsigned long PULSE_GAP_MS = 100; // wait after a pulse before checking

// continuous cascade gains. Wobbles: lower RATE_KP. Creeps: raise ANGLE_KP.
static const unsigned long LOOP_MS = 10;
static const float ANGLE_KP = 2.0;      // deg/s back towards target per deg off
static const float ANGLE_KD = 0.2;      // less return speed while already moving
static const float MAX_RETURN = 20.0;   // deg/s, fastest it turns back
static const float RATE_KP = 0.08;      // share of the rate error added every 10 ms
static const float RATE_KD = 0.3;       // damping on the change of rate error
static const float MAX_WHEEL = 50.0;    // deg/s, wheel limit (motor clamps ~54)
static const byte AVG_N = 5;            // gyro rate moving average

static SteadyState state = STEADY_OFF;
static unsigned long stateStart = 0;
static float holdAngle = 0;
static float drift = 0;
static float correction = 0;
static float rateSum = 0;
static long rateN = 0;
static float wheelCmd = 0;              // hold speed now, body deg/s
static unsigned long lastLoopMs = 0;
static float rateBuf[AVG_N];
static byte rateIdx = 0;
static float lastRateErr = 0;
static bool pulsing = false;            // a pulse turn is running
static unsigned long pulseEndMs = 0;

static void enter(SteadyState s) {
  state = s;
  stateStart = millis();
}

static void resetCascade() {
  for (byte i = 0; i < AVG_N; i++) rateBuf[i] = gyroRate();
  rateIdx = 0;
  lastRateErr = 0;
  lastLoopMs = millis();
}

// continuous hold, every 10 ms:
//   outer (angle PD): wanted body rate back towards the target
//   inner (rate PD, velocity form): adds to the wheel speed until the body
//   moves at the wanted rate - keeps speeding the wheel up against a wire
static void holdLoop() {
  if (millis() - lastLoopMs < LOOP_MS) return;
  lastLoopMs = millis();

  rateBuf[rateIdx] = gyroRate();
  rateIdx = (rateIdx + 1) % AVG_N;
  float rate = 0;
  for (byte i = 0; i < AVG_N; i++) rate += rateBuf[i];
  rate /= AVG_N;

  float want = ANGLE_KP * (holdAngle - gyroAngle()) - ANGLE_KD * rate;
  want = constrain(want, -MAX_RETURN, MAX_RETURN);

  float rateErr = want - rate;
  wheelCmd += RATE_KP * rateErr + RATE_KD * (rateErr - lastRateErr);
  lastRateErr = rateErr;
  wheelCmd = constrain(wheelCmd, -MAX_WHEEL, MAX_WHEEL);
  motorHoldSpeed(wheelCmd);
}

// pulse hold: off by more than PULSE_DEAD -> small turn back, brake, wait
static void pulseLoop() {
  if (motorBusy()) return;               // pulse still running
  if (pulsing) {
    pulsing = false;
    pulseEndMs = millis();
  }
  if (millis() - pulseEndMs < PULSE_GAP_MS) return;   // let it settle
  float err = holdAngle - gyroAngle();
  if (fabs(err) < PULSE_DEAD) return;
  turnBy(err);
  pulsing = true;
}

static void runHold() {
  if (HOLD_MODE == HOLD_PULSE) pulseLoop();
  if (HOLD_MODE == HOLD_CONTINUOUS) holdLoop();
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
      if (t < SETTLE_MS) break;
      if (HOLD_MODE == HOLD_PULSE) {     // no drift measuring, pulse straight away
        correction = holdAngle - gyroAngle();
        pulsing = false;
        pulseEndMs = millis() - PULSE_GAP_MS;
        enter(STEADY_RETURN);
      } else {
        enter(STEADY_MEASURE);
      }
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
        if (HOLD_MODE == HOLD_ONCE) motorHoldNudge(correction);   // else holdLoop returns it
        resetCascade();
        enter(STEADY_RETURN);
      }
      break;

    case STEADY_RETURN: {
      runHold();
      float off = fabs(holdAngle - gyroAngle());
      bool there = HOLD_MODE == HOLD_PULSE ? (!motorBusy() && off < PULSE_DEAD)
                                           : off < RETURN_TOL;
      if (there || t >= RETURN_MS) enter(STEADY_READY);
      break;
    }

    case STEADY_READY:
      runHold();                         // keep it on target
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
