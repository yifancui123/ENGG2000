#include "Arduino.h"
#include "motor_control.h"

const int motorPWM = 5;
const int motorDirection = 6;
const int encoderA = 2;
const int encoderB = 3;
const int sleep = 7;
const int turnSpeed = 150;
float countsPerDegree = 700.0 / 360.0;
volatile int pos = 0;

const long MS_PER_SCHEDULED_INVERT = 20000;
long ms_at_last_scheduled_invert = 0;
bool rotating_right = true;

void readEncoder() {
  int b = digitalRead(encoderB);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}

void motorForward(int speedValue) {
  digitalWrite(motorDirection, HIGH);
  analogWrite(motorPWM, speedValue);
}

void motorReverse(int speedValue) {
  digitalWrite(motorDirection, LOW);
  analogWrite(motorPWM, speedValue);
}

void motorStop() {
  analogWrite(motorPWM, 0);
}

void turnAngle(float angle, int speedValue) {
  long targetCount = abs(angle) * countsPerDegree;

  noInterrupts();
  pos = 0;
  interrupts();

  if (angle > 0) {
    motorForward(speedValue);
  } else if (angle < 0) {
    motorReverse(speedValue);
  } else {
    return;
  }

  unsigned long startTime = millis();
  while (abs(pos) < targetCount) {
    if (millis() - startTime > 5000) break; // safety timeout
  }

  motorStop();
}

void rotateTo(float angle) {
  turnAngle(angle, turnSpeed);
}

bool isAtTarget() {
  return true;
}

void setupMotor() {
  pinMode(sleep, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  digitalWrite(sleep, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);
}

void invert_direction(){
  rotating_right = !rotating_right;
}

void try_invert(){
  long ms_since_scheduled_invert = millis() - ms_at_last_scheduled_invert;
  if (ms_since_scheduled_invert > MS_PER_SCHEDULED_INVERT){
    invert_direction();
  }
}
