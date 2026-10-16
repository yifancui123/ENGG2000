// driver.h - DRV8874 motor driver helper shared by the test sketches.
// The same file is copied into every sketch folder; keep the copies identical.
//
// The driver can end up set up three ways, and each needs different pin logic:
//   0 = PH/EN mode as designed: D5 = EN (speed), D6 = PH (direction)
//   1 = IN1/IN2 mode (PMODE pin high): D5 = IN1, D6 = IN2
//   2 = PH/EN mode with the D5 and D6 wires swapped: D6 = EN, D5 = PH
// test_driver_mode works out which one this board is and saves it to EEPROM.
// Every other sketch reads it from there and refuses to run the motor until set.

#pragma once
#include <Arduino.h>
#include <EEPROM.h>

const int PIN_DRV_A = 5;           // EN in mode 0, IN1 in mode 1, PH in mode 2
const int PIN_DRV_B = 6;           // PH in mode 0, IN2 in mode 1, EN in mode 2
const int PIN_SLEEP = 7;           // DRV8874 nSLEEP: LOW = driver off (coast)

const int     MODE_ADDR  = 1000;   // EEPROM: far away from test_satellite's results
const uint8_t MODE_MAGIC = 0xA5;

int driverMode = -1;               // -1 = unknown: motor stays off

const char* modeName(int m) {
  switch (m) {
    case 0:  return "PH/EN (as designed)";
    case 1:  return "IN1/IN2 (PMODE high)";
    case 2:  return "PH/EN with D5/D6 swapped";
    default: return "NOT SET - run test_driver_mode first";
  }
}

int readDriverMode() {
  if (EEPROM.read(MODE_ADDR) != MODE_MAGIC) return -1;
  uint8_t m = EEPROM.read(MODE_ADDR + 1);
  return m <= 2 ? m : -1;
}

void saveDriverMode(int m) {
  EEPROM.update(MODE_ADDR, MODE_MAGIC);
  EEPROM.update(MODE_ADDR + 1, (uint8_t)m);
}

// Both inputs LOW never drives the motor in any of the three modes.
void driverInputsLow() {
  digitalWrite(PIN_DRV_A, LOW);
  digitalWrite(PIN_DRV_B, LOW);
}

// Driver off: outputs float, the wheel spins down freely.
void coast() {
  driverInputsLow();
  digitalWrite(PIN_SLEEP, LOW);
}

void wake() {
  if (digitalRead(PIN_SLEEP) == LOW) {
    digitalWrite(PIN_SLEEP, HIGH);
    delay(2);                      // DRV8874 wake-up time
  }
}

// Short the motor so the wheel stops quickly.
void brake() {
  if (driverMode < 0) { coast(); return; }
  if (driverMode == 1) {           // IN1/IN2: both HIGH = brake
    digitalWrite(PIN_DRV_A, HIGH);
    digitalWrite(PIN_DRV_B, HIGH);
  } else {                         // PH/EN: EN LOW = brake (EN is A or B)
    driverInputsLow();
  }
  wake();
}

// Signed PWM, -255..255. Positive = OUT1 high (same polarity in every mode).
// In every mode the PWM off-time brakes the motor, so speed vs PWM matches.
void drive(int pwm) {
  if (driverMode < 0) { coast(); return; }
  pwm = constrain(pwm, -255, 255);
  int mag = abs(pwm);
  bool fwd = pwm >= 0;
  wake();
  switch (driverMode) {
    case 0:
      digitalWrite(PIN_DRV_B, fwd ? HIGH : LOW);
      analogWrite(PIN_DRV_A, mag);
      break;
    case 2:
      digitalWrite(PIN_DRV_A, fwd ? HIGH : LOW);
      analogWrite(PIN_DRV_B, mag);
      break;
    case 1:                        // one input HIGH, PWM the other inverted
      if (fwd) { digitalWrite(PIN_DRV_A, HIGH); analogWrite(PIN_DRV_B, 255 - mag); }
      else     { digitalWrite(PIN_DRV_B, HIGH); analogWrite(PIN_DRV_A, 255 - mag); }
      break;
  }
}

// Call in setup() after Serial.begin(). Returns false if the mode isn't set yet.
bool driverBegin() {
  pinMode(PIN_DRV_A, OUTPUT);
  pinMode(PIN_DRV_B, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  coast();
  driverMode = readDriverMode();
  Serial.print(F("# driver mode: "));
  Serial.println(modeName(driverMode));
  if (driverMode < 0) return false;
  brake();
  return true;
}
