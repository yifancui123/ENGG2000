// test_speed_curve - steady wheel speed at each PWM level, both directions.
//
// Starts automatically 3 s after reset. For each PWM it waits 1.5 s for the
// flywheel to reach speed, then measures counts over 1.0 s.
// Prints CSV: pwm,rpm (pwm is signed). Takes ~1.5 minutes.

#include "driver.h"

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

const float COUNTS_PER_REV = 2800.0;   // 700 x 4 - confirm with test_encoder_check
const int   PWM_LEVELS[]   = {40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 255};
const int   N_LEVELS       = sizeof(PWM_LEVELS) / sizeof(PWM_LEVELS[0]);
const unsigned long SETTLE_MS  = 1500;
const unsigned long MEASURE_MS = 1000;

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

  Serial.println(F("# speed curve starts in 3 s - keep hands clear"));
  delay(3000);
  Serial.println(F("pwm,rpm"));

  for (int dir = 1; dir >= -1; dir -= 2) {
    for (int i = 0; i < N_LEVELS; i++) {
      int pwm = dir * PWM_LEVELS[i];
      drive(pwm);
      delay(SETTLE_MS);
      long before = readCount();
      delay(MEASURE_MS);
      long moved = readCount() - before;
      float rpm = moved / COUNTS_PER_REV / (MEASURE_MS / 1000.0) * 60.0;
      Serial.print(pwm); Serial.print(',');
      Serial.println(rpm, 1);
    }
    brake();
    delay(3000);                               // let the flywheel stop
  }
  Serial.println(F("# end"));
}

void loop() {}
