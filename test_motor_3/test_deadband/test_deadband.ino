// test_deadband - lowest PWM that starts the wheel (breakaway) and the PWM
// where it stops again (kinetic), both directions, 3 trials each.
//
// Starts automatically 3 s after reset (opening Serial Monitor resets the Nano).
// Ramps PWM up 1 step every 100 ms until the wheel moves, then ramps down
// until it stops. Prints a CSV summary at the end. Takes ~3-4 minutes.

#include "driver.h"

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

const int  STEP_MS        = 100;  // time at each PWM level
const long MOVE_COUNTS    = 6;    // > this many counts in STEP_MS = "moving"
const int  TRIALS         = 3;

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

long countsIn(int pwm) {                    // run one step, return |counts moved|
  long before = readCount();
  drive(pwm);
  delay(STEP_MS);
  return labs(readCount() - before);
}

void setup() {
  Serial.begin(115200);
  if (!driverBegin()) {
    Serial.println(F("# motor disabled: upload test_driver_mode and send 'd' first"));
    Serial.println(F("# end"));
    return;
  }
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  prevAB = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  Serial.println(F("# deadband test starts in 3 s - keep hands clear of the flywheel"));
  delay(3000);

  Serial.println(F("direction,trial,breakaway_pwm,stop_pwm"));
  for (int dir = 1; dir >= -1; dir -= 2) {
    for (int trial = 1; trial <= TRIALS; trial++) {
      int pwm = 0;
      while (pwm < 255 && countsIn(dir * pwm) <= MOVE_COUNTS) pwm++;
      int breakaway = pwm;

      while (pwm > 0 && countsIn(dir * pwm) > MOVE_COUNTS) pwm--;
      int stopPwm = pwm;

      brake();
      Serial.print(dir > 0 ? F("forward,") : F("reverse,"));
      Serial.print(trial); Serial.print(',');
      Serial.print(breakaway); Serial.print(',');
      Serial.println(stopPwm);
      delay(2000);
    }
  }
  Serial.println(F("# end"));
}

void loop() {}
