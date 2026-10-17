#include "Arduino.h"
#include "motor_control.h"
#include <math.h>

const int motorPWM = 5;
const int motorDirection = 6;
const int encoderA = 2;
const int encoderB = 3;
const int sleep = 7;
const int turnSpeed = 150;
float countsPerDegree = 700.0 / 360.0;
volatile int pos = 0;

// ---- brake tuning ----   ★ = tune on the bench
const float BRAKE_KD      = 30.0f; // ★ PWM per (count/ms) of speed
const int   BRAKE_MINPWM  = 40;    // ★ smallest PWM that actually moves the motor (stiction)
const int   BRAKE_MAXPWM  = 200;   //    clamp so a fast spin doesn't slam full power
const float STOP_SPEED    = 0.15f; // ★ counts/ms below which we call it "stopped"
const int   SAMPLE_MS     = 10;    //    how often we measure speed
const int   SETTLE_NEEDED = 5;     //    consecutive "stopped" samples before we trust it
const bool  BRAKE_DEBUG   = true;  //    print vel while tuning; set false later

// Atomic read of the volatile encoder count.
// (int is 16-bit on AVR -> a plain read can "tear" if the ISR fires mid-read.)
static int readPos() {
  noInterrupts();
  int p = pos;
  interrupts();
  return p;
}

// Actively brake the reaction wheel: damp the encoder velocity to ~0.
// Returns when stopped for SETTLE_NEEDED samples, or after maxMs.
void brakeHold(unsigned long maxMs) {
  unsigned long tStart = millis();
  int           lastPos = readPos();
  unsigned long lastT   = millis();
  int           settleCount = 0;

  while (millis() - tStart < maxMs) {
    delay(SAMPLE_MS);

    int           nowPos = readPos();
    unsigned long nowT   = millis();
    float dt  = (float)(nowT - lastT);      // ms
    float vel = (nowPos - lastPos) / dt;    // counts/ms, signed

    lastPos = nowPos;
    lastT   = nowT;

    if (BRAKE_DEBUG) {
      Serial.print("vel=");
      Serial.println(vel);
    }

    // deadband: slow enough -> call it stopped, don't keep twitching
    if (fabs(vel) < STOP_SPEED) {
      motorStop();
      settleCount++;
      if (settleCount >= SETTLE_NEEDED) return;   // stopped and stayed stopped
      continue;
    }
    settleCount = 0;

    // brake: push opposite to the motion, harder when faster
    int drive = (int)(BRAKE_KD * fabs(vel));
    drive = constrain(drive, BRAKE_MINPWM, BRAKE_MAXPWM);

    if (vel > 0) motorReverse(drive);   // ★ if it speeds up instead of slowing, swap these two lines
    else         motorForward(drive);
  }
  motorStop();   // timed out
}

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
