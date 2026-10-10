#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern volatile int pos;
extern const long MS_PER_SCHEDULED_INVERT;
extern long ms_at_last_scheduled_invert;
extern bool rotating_right;



void motorForward(int speedValue);
void motorReverse(int speedValue);
void motorStop();
void turnAngle(float angle, int speedValue);
void rotateTo(float angle);
bool isAtTarget();
void setupMotor(); 

void invert_direction();
void try_invert();


#endif
