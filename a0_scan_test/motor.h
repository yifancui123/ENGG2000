// ===== motor.h - reaction wheel turning for MQ Sentinel =====
// Turns the satellite body by a number of degrees using the reaction wheel.
// The wheel position is held by a PD controller on the encoder count.
//
// Use:
//   setup():  motorBegin();
//   loop():   motorUpdate();            // EVERY loop, keep loop() fast
//             if (!motorBusy()) turnBy(err);
//
// Pins: D2 = encoder A, D3 = encoder B (interrupts), D5 = EN, D6 = PH,
//       D7 = nSLEEP. Driver PMODE must be tied to GND.

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

enum MotorResult {
  MOTOR_IDLE,      // no move started yet
  MOTOR_BUSY,      // move running
  MOTOR_DONE,      // reached the target and stopped
  MOTOR_TIMEOUT,   // took too long, braked and gave up
  MOTOR_STOPPED,   // cancelled with motorStop()
  MOTOR_HOLDING    // wheel turning at a steady speed (motorHold)
};

void motorBegin();                // call once in setup()
void turnBy(float bodyDeg);       // + = anticlockwise (left), - = clockwise (right), seen from above
void moveWheelBy(long counts);    // raw wheel move in encoder counts (2800 = 1 wheel rev)
void motorUpdate();               // runs the PD - call every loop()
void motorHold(float bodyDegPerSec); // keep the wheel at a steady speed (cancel drift: use -drift)
void motorHoldNudge(float bodyDeg); // while holding: turn the body by bodyDeg on top of the hold
void motorHoldSpeed(float bodyDegPerSec); // while holding: change the speed smoothly
bool motorBusy();                 // true while a move is running (false while holding)
bool motorHolding();              // true while motorHold is running
MotorResult motorResult();        // how the last move ended
void motorStop();                 // brake now and cancel the move

long motorCount();                // wheel position (counts), for logging
long motorTarget();               // target of the current/last move (counts)
int motorPwm();                   // last PWM sent, -255..255, for logging

#endif
