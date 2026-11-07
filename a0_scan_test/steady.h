// ===== steady.h - turn with the gyro, then drift hold =====
// steadyTurnBy(deg) does the whole job:
//   first turn (wheel at rest):
//     TURNING  turnBy(deg) on the encoder; target = gyro angle now + deg
//     SETTLE   wheel braked, 0.3 s
//     MEASURE  let it drift 1 s, gyro measures the drift rate
//     RETURN   wheel spins at the speed that cancels it, turn back onto target
//   later turns: RETURN straight away, the turn runs on top of the spinning
//     wheel (no brake, no jerk)
//   READY      on target -> fire the laser. Every 0.5 s the drift that is
//              left is re-measured and the wheel speed adjusted; more than
//              0.3 deg off -> turned back.
//
// Use:
//   loop(): motorUpdate(); gyroUpdate(); steadyUpdate();   // every loop
//   steadyTurnBy(errDeg);                 // start a turn
//   if (steadyReady()) ... laser on target, count the 2 s ...
//   steadyStop();  then steadyTurnBy(next)

#ifndef STEADY_H
#define STEADY_H

#include <Arduino.h>

enum SteadyState {
  STEADY_OFF,
  STEADY_TURNING,
  STEADY_SETTLE,
  STEADY_MEASURE,
  STEADY_RETURN,
  STEADY_READY
};

void steadyTurnBy(float deg);    // turn by deg (+ = anticlockwise), then hold there
void steadyStart();              // hold where the body is now
void steadyStartAt(float angle); // hold this gyro angle
void steadyUpdate();             // every loop (needs motorUpdate + gyroUpdate too)
void steadyStop();               // clear the state (the wheel keeps its speed)
bool steadyActive();             // anything but STEADY_OFF
bool steadyReady();              // holding on the target
SteadyState steadyState();

float steadyHoldAngle();         // the target it holds (gyro deg)
float steadyDrift();             // drift measured after the first turn, deg/s
float steadyCorrection();        // angle turned back after measuring, deg

#endif
