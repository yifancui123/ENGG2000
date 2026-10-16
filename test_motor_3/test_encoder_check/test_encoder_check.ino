// test_encoder_check - verify counts per wheel revolution and direction.
//
// Motor is OFF unless you send a command. Serial Monitor at 115200.
//   z  -> zero the count
//   f  -> short forward pulse  (PWM 100 for 300 ms)
//   r  -> short reverse pulse
//   1  -> motor turns the wheel ONE revolution (2800 counts) and stops
//
// No need to touch the flywheel - the motor does all the turning.
// 1. Send 'f' and 'r' a few times. 'f' should move a clearly POSITIVE count
//    (hundreds or more), 'r' a similar negative one. If 'f' goes negative,
//    swap the D2 and D3 wires. If the wheel spins but the count stays near 0,
//    one encoder wire is loose.
// 2. Battery off, put a tape mark on the flywheel rim and one on the frame
//    next to it. Battery on, hands clear, send '1'. The marks should line up
//    again; the sketch prints how far past the start mark it should stop.
// Needs the driver mode saved by test_driver_mode (run that first).

#include "driver.h"

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;

volatile long count = 0;
volatile uint8_t prevAB = 0;
const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void encoderISR() {                         // 4x quadrature decode, D2 = A, D3 = B
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

const long ONE_REV = 2800;                  // 700 x 4 (datasheet) - what we check

// Turn the wheel one revolution under motor power, then report where it stopped.
// Fast (PWM 120) for the first 3/4 rev, slow (PWM 70) for the last 1/4 so it
// doesn't overshoot much. If the wheel stalls, the PWM is raised 10 at a time.
void moveOneRev() {
  Serial.println(F("# moving one revolution..."));
  long start = readCount();
  long lastMoved = 0;
  int boost = 0;
  unsigned long t0 = millis(), lastChange = millis();

  while (true) {
    long moved = readCount() - start;
    if (moved >= ONE_REV) break;
    if (moved < -50) {
      brake();
      Serial.println(F("# STOPPED: count went DOWN while driving forward - swap D2 and D3"));
      return;
    }
    if (millis() - t0 > 10000) {
      brake();
      Serial.print(F("# STOPPED: timed out after 10 s, moved only "));
      Serial.print(moved); Serial.println(F(" counts - check battery and encoder"));
      return;
    }
    if (moved != lastMoved) {
      lastMoved = moved;
      lastChange = millis();
    } else if (millis() - lastChange > 200 && boost < 100) {
      boost += 10;                          // stalled: push a bit harder
      lastChange = millis();
    }
    int pwm = (ONE_REV - moved > ONE_REV / 4) ? 120 : 70;
    drive(min(pwm + boost, 220));
  }
  brake();
  delay(1000);                              // let the flywheel settle

  long moved = readCount() - start;
  float pastDeg = (moved - ONE_REV) * 360.0 / ONE_REV;
  float pastMm  = pastDeg / 360.0 * PI * 100.0;   // 100 mm flywheel rim
  Serial.print(F("# moved ")); Serial.print(moved);
  Serial.print(F(" counts. If 2800 is right, the tape mark is "));
  Serial.print(pastDeg, 1); Serial.print(F(" deg ("));
  Serial.print(pastMm, 0); Serial.println(F(" mm at the rim) PAST the frame mark."));
}

void setup() {
  Serial.begin(115200);
  driverBegin();
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  prevAB = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
  Serial.println(F("# encoder check: z=zero, f=forward pulse, r=reverse pulse, 1=one revolution"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { noInterrupts(); count = 0; interrupts(); }
    if ((c == '1' || c == 'f' || c == 'r') && driverMode < 0) {
      Serial.println(F("# motor disabled: upload test_driver_mode and send 'd' first"));
      return;
    }
    if (c == '1') moveOneRev();
    if (c == 'f' || c == 'r') {
      long before = readCount();
      drive(c == 'f' ? 100 : -100);
      delay(300);
      brake();
      delay(500);
      Serial.print(F("# pulse '")); Serial.print(c);
      Serial.print(F("' moved ")); Serial.print(readCount() - before);
      Serial.println(F(" counts"));
    }
  }

  static unsigned long last = 0;
  if (millis() - last >= 200) {
    last = millis();
    Serial.print(F("count: "));
    Serial.println(readCount());
  }
}
