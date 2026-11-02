// ===== steady.h - turn with the gyro, then measure-once drift cancel =====
// steadyTurnBy(deg) does the whole job:
//   0. TURNING  turnBy(deg) on the encoder; target = gyro angle now + deg
//   1. SETTLE   turn finished, wait a moment for the brake
//   2. MEASURE  wheel braked, gyro measures the body drift rate
//   3. CANCEL   wheel set to the steady speed that cancels that drift
//   4. RETURN   turn the body onto the target angle (fixes turn error AND
//               the angle lost while measuring)
//   5. READY    holding on the target -> fire the laser
// With CONTINUOUS = true (steady.cpp) steps 4 and 5 keep correcting the
// wheel speed with the gyro every 20 ms, so a wire that twists back is
// fought for the whole hold. steadyStop() + the next steadyTurnBy()
// clears it and the next target measures again.
//
// Use:
//   loop(): motorUpdate(); gyroUpdate(); steadyUpdate();   // every loop
//   steadyTurnBy(errDeg);                 // start a turn (when !steadyActive())
//   if (steadyReady()) ... laser on target, count the 2 s ...
//   steadyStop();  then steadyTurnBy(next)  (the next turn clears the speed)

#ifndef STEADY_H
#define STEADY_H

#include <Arduino.h>

enum SteadyState {
  STEADY_OFF,
  STEADY_TURNING,
  STEADY_SETTLE,
  STEADY_MEASURE,
  STEADY_CANCEL,
  STEADY_RETURN,
  STEADY_READY
};

void steadyTurnBy(float deg);    // turn by deg (+ = anticlockwise), then hold there
void steadyStart();              // hold where the body is now (turn already done)
void steadyStartAt(float angle); // hold this gyro angle (turn already done)
void steadyUpdate();             // every loop (needs motorUpdate + gyroUpdate too)
void steadyStop();               // clear the state (next turn clears the speed)
bool steadyActive();             // anything but STEADY_OFF
bool steadyReady();              // holding on the target
SteadyState steadyState();

float steadyHoldAngle();         // the target it holds (gyro deg)
float steadyDrift();             // drift it measured, deg/s
float steadyCorrection();        // angle it turned to get onto the target, deg

#endif
