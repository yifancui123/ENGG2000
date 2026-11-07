// Motor test with the encoder only - NO gyro (not part of the project)
// motor.h / motor.cpp are unchanged copies from main_see_fire/PD.
// Serial Monitor 115200, New Line. Type a body angle, e.g. 90 or -90:
// the wheel turns 90 / 0.14 = ~643 deg (5000 counts) and stops.
// + should turn the body anticlockwise seen from above (when hanging).
// s = stop (brake)

#include "motor.h"

static unsigned long lastPrintMs = 0;
static bool wasBusy = false;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  motorBegin();
  Serial.println(F("ready: type a body angle, e.g. 90 or -90 (s = stop)"));
}

void loop() {
  motorUpdate();                          // EVERY loop

  if (Serial.available()) {
    char c = Serial.peek();
    if (c == 's') {
      Serial.read();
      motorStop();
      Serial.println(F("stopped"));
    } else if (isDigit(c) || c == '-' || c == '+') {
      float deg = Serial.parseFloat();
      Serial.print(F("turnBy ")); Serial.println(deg, 1);
      turnBy(deg);
    } else {
      Serial.read();                      // ignore newlines
    }
  }

  bool busy = motorBusy();
  if (busy && millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();
    Serial.print(F("  count ")); Serial.print(motorCount());
    Serial.print(F("  target ")); Serial.print(motorTarget());
    Serial.print(F("  pwm ")); Serial.println(motorPwm());
  }
  if (wasBusy && !busy) {
    MotorResult r = motorResult();
    Serial.print(r == MOTOR_DONE ? F("DONE") : r == MOTOR_TIMEOUT ? F("TIMEOUT") : F("STOPPED"));
    Serial.print(F("  count ")); Serial.print(motorCount());
    Serial.print(F("  target ")); Serial.println(motorTarget());
  }
  wasBusy = busy;
}
