#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern volatile long pos;

void motorForward(int speedValue);
void motorReverse(int speedValue);
void motorStop();
long readPos();
bool moveTo(long target, int maxSpeed, unsigned long timeoutMs);
bool stepDegrees(float angle, int maxSpeed);
void turnAngle(float angle, int speedValue);
void rotateTo(float angle);
bool isAtTarget();
void setupMotor();
#endif
