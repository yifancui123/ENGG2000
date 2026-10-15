// test_encoder_check - verify counts per wheel revolution and direction.
//
// Motor is OFF unless you send a command. Serial Monitor at 115200.
//   z  -> zero the count
//   f  -> short forward pulse  (PWM 100 for 300 ms)
//   r  -> short reverse pulse
//
// 1. Put a tape mark on the flywheel. Send 'z'.
// 2. Turn the flywheel by hand exactly 5 full turns (slowly).
// 3. counts / 5 = counts per rev. Expect ~2800 (700 x 4: every edge on A and B).
// 4. Send 'f': the count should go UP. If it goes down, note it -
//    the other sketches print FORWARD_SIGN so you can flip it there.

const int PIN_ENC_A = 2;
const int PIN_ENC_B = 3;
const int PIN_EN    = 5;   // DRV8874 EN (PWM)
const int PIN_PH    = 6;   // DRV8874 PH (direction)
const int PIN_SLEEP = 7;   // DRV8874 nSLEEP

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

void drive(int pwm) {                       // signed PWM, -255..255
  digitalWrite(PIN_PH, pwm >= 0 ? HIGH : LOW);
  analogWrite(PIN_EN, constrain(abs(pwm), 0, 255));
}

void brake() { analogWrite(PIN_EN, 0); }    // DRV8874 PH/EN: EN low = brake

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PH, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  digitalWrite(PIN_SLEEP, HIGH);
  brake();
  prevAB = ((PIND >> 2) & 1) << 1 | ((PIND >> 3) & 1);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);
  Serial.println(F("# encoder check: z=zero, f=forward pulse, r=reverse pulse"));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') { noInterrupts(); count = 0; interrupts(); }
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
