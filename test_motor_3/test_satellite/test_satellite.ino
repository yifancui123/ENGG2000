// test_satellite - run on the HANGING satellite, on BATTERY, USB unplugged.
// Use with the phone (phyphox "Gyroscope") taped on top, recording.
//
// Power on -> 15 s countdown (pin 13 LED blinks) -> 6 moves -> LED stays on.
// Each move: ramp the wheel to PWM 120 over 0.3 s, hold 1.2 s, BRAKE,
// then wait 8 s so you can see whether the body stops or keeps drifting.
// Moves alternate direction: + - + - + -
// Wheel counts moved in each move are saved to EEPROM.
//
// Reading results: plug USB in, open Serial Monitor (this resets the Nano),
// and send 'p' WITHIN the 15 s countdown. It prints the results and does
// NOT spin the motor. If you miss the window, the test runs again.
//
// Inertia ratio: body angle (phone) / wheel angle (counts / 2800 * 360)

#include <EEPROM.h>
#include "driver.h"

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;
const int PIN_LED   = 13;

const int   N_MOVES    = 6;
const int   MOVE_PWM   = 120;
const float COUNTS_PER_REV = 2800.0;
const uint16_t MAGIC   = 0x5A3C;            // marks valid saved data

volatile long count = 0;
volatile uint8_t prevAB = 0;
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

struct Result {
  uint16_t magic;
  long moved[N_MOVES];      // counts from start of move to end of 8 s wait
  long peakRate[N_MOVES];   // max counts per 100 ms during the move
};

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

void printResults() {
  Result r;
  EEPROM.get(0, r);
  if (r.magic != MAGIC) {
    Serial.println(F("# no saved results"));
    return;
  }
  Serial.println(F("move,direction,wheel_counts,wheel_deg,peak_counts_per_100ms"));
  for (int i = 0; i < N_MOVES; i++) {
    Serial.print(i + 1); Serial.print(',');
    Serial.print(i % 2 == 0 ? F("+") : F("-")); Serial.print(',');
    Serial.print(r.moved[i]); Serial.print(',');
    Serial.print(r.moved[i] / COUNTS_PER_REV * 360.0, 1); Serial.print(',');
    Serial.println(r.peakRate[i]);
  }
  Serial.println(F("# end"));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  if (!driverBegin()) {             // mode not saved: fast blink, never run
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(80); }
  }
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  prevAB = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  // 15 s countdown: send 'p' to print saved results instead of running
  Serial.println(F("# send 'p' within 15 s to print results (motor will not run)"));
  for (int i = 0; i < 150; i++) {
    digitalWrite(PIN_LED, (i / 5) % 2);
    if (Serial.available() && Serial.read() == 'p') {
      digitalWrite(PIN_LED, LOW);
      printResults();
      while (true) {}
    }
    delay(100);
  }

  Result r;
  r.magic = MAGIC;
  for (int m = 0; m < N_MOVES; m++) {
    int dir = (m % 2 == 0) ? 1 : -1;
    long start = readCount(), last = start, peak = 0;
    digitalWrite(PIN_LED, HIGH);

    for (int k = 1; k <= 15; k++) {          // smooth ramp up over 0.3 s
      drive(dir * MOVE_PWM * k / 15);
      delay(20);
    }
    for (int k = 0; k < 12; k++) {           // hold 1.2 s, track peak speed
      delay(100);
      long now = readCount();
      peak = max(peak, labs(now - last));
      last = now;
    }
    brake();
    digitalWrite(PIN_LED, LOW);
    delay(8000);                             // watch the body settle

    r.moved[m] = readCount() - start;
    r.peakRate[m] = peak;
  }
  EEPROM.put(0, r);
  digitalWrite(PIN_LED, HIGH);               // done
}

void loop() {}
