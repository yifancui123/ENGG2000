#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern volatile long encoderCount;

void motorForward(int speedValue);
void motorReverse(int speedValue);
void motorStop();
void turnAngle(float angle);
void rotateTo(float angle);
bool isAtTarget();
void setupMotor(); 
#endif