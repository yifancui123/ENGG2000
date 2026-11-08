// ===== pd_hold.h - encoder PD turns + gyro hold (from simple_pd_fire) =====
// Turning: the encoder PD turns the wheel by the counts for the wanted body
// angle; the gyro says where the body really is.
// Hold: every CHECK_MS after a move, if the body is more than HOLD_TOL off
// the hold angle, a small encoder move back, then brake. It holds all the time.
//
// Use:
//   setup(): pdBegin();  ... gyroCalibrate ...;  pdHoldHere();
//   loop():  gyroUpdate(); pdUpdate();          // every loop
//            pdTurnBy(deg);                     // + = anticlockwise (left)
//            if (pdStill()) ... sensors can be trusted ...
//
// Wiring: encoder A D2, B D3; driver EN D5, PH D6, nSLEEP D7, PMODE -> GND.

#ifndef PD_HOLD_H
#define PD_HOLD_H

#include <Arduino.h>

void pdBegin();                   // motor pins, brake, encoder interrupts
void pdHoldHere();                // hold the gyro angle the body is at now
void pdStopHere();                // stop any move now (brake) and hold where it is
void pdTurnBy(float deg);         // turn the body by deg, then hold there
void pdUpdate();                  // runs the PD and the hold, every loop
bool pdStill();                   // on the hold angle and not moving
unsigned long pdStillSinceMs();   // when it last became still
float pdHoldAngle();              // the angle it holds, gyro deg

#endif
