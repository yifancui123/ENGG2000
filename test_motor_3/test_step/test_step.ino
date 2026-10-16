// test_step - step response + stopping behaviour (brake or coast).
//
// Send a command in Serial Monitor (115200, "Newline" line ending):
//   150    -> PWM +150 step, then BRAKE
//   -150   -> reverse step, then BRAKE
//   c150   -> PWM +150 step, then COAST (driver asleep, wheel spins down freely)
// Each run logs t_ms,count,pwm every 5 ms:
//   0.2 s at rest, 2.5 s at the step PWM, then 3 s (brake) or 8 s (coast) stopping.
// PWM 255 gives the maximum speed on the current battery voltage.
// Easiest: let MATLAB run it (matlab/run_motor_tests.m) so logs save as CSV.

#include "driver.h"

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

const unsigned long LOG_US   = 5000;   // 5 ms sample period
const unsigned long T_REST   = 200;    // ms before the step
const unsigned long T_STEP   = 2500;   // ms at the step PWM
const unsigned long T_BRAKE  = 3000;   // ms logged after a brake stop
const unsigned long T_COAST  = 8000;   // ms logged after a coast stop (slow)

volatile long count = 0;
volatile uint8_t prevAB = 0;
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void encoderISR() {
  uint8_t ab = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  count += QDEC[(prevAB << 2) | ab];
  prevAB = ab;
}

long readCount() {
  noInterrupts();
  long c = count;
  interrupts();
  return c;
}

void runStep(int pwm, bool useCoast) {
  Serial.print(F("# step pwm=")); Serial.print(pwm);
  Serial.println(useCoast ? F(" stop=coast") : F(" stop=brake"));
  Serial.println(F("t_ms,count,pwm"));

  noInterrupts(); count = 0; interrupts();
  brake();
  unsigned long t0 = micros(), next = t0;
  bool stepped = false, stopped = false;
  int nowPwm = 0;
  unsigned long tStop = useCoast ? T_COAST : T_BRAKE;

  while (true) {
    unsigned long t = micros() - t0;
    if (!stepped && t >= T_REST * 1000UL) { drive(pwm); nowPwm = pwm; stepped = true; }
    if (!stopped && t >= (T_REST + T_STEP) * 1000UL) {
      if (useCoast) coast(); else brake();
      nowPwm = 0; stopped = true;
    }
    if (t >= (T_REST + T_STEP + tStop) * 1000UL) break;

    if ((long)(micros() - next) >= 0) {
      next += LOG_US;
      Serial.print(t / 1000.0, 1); Serial.print(',');
      Serial.print(readCount()); Serial.print(',');
      Serial.println(nowPwm);
    }
  }
  brake();
  Serial.println(F("# end"));
}

void setup() {
  Serial.begin(115200);
  driverBegin();
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  prevAB = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
  Serial.println(F("# ready: send 150, -150 or c150"));
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  bool useCoast = (cmd[0] == 'c');
  if (useCoast) cmd.remove(0, 1);
  if (driverMode < 0) {
    Serial.println(F("# motor disabled: upload test_driver_mode and send 'd' first"));
    Serial.println(F("# end"));
    return;
  }
  int pwm = constrain(cmd.toInt(), -255, 255);
  if (pwm != 0) runStep(pwm, useCoast);
}
