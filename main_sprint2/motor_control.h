#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern volatile int pos;

void motorForward(int speedValue);
void motorReverse(int speedValue);
void motorStop();
void turnAngle(float angle, int speedValue);
void rotateTo(float angle);
bool isAtTarget();
void setupMotor(); 
#endif