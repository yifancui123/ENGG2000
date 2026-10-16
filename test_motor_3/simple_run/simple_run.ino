// simple_run - just runs the motor. No setup needed.
// Power on -> waits 5 s (LED on) -> forward 2 s -> brake 3 s -> reverse 2 s -> brake 3 s -> repeats.
// Driver in PH/EN mode (PMODE grounded): D5 = EN (speed), D6 = PH (direction), D7 = nSLEEP.

const int PIN_EN    = 5;
const int PIN_PH    = 6;
const int PIN_SLEEP = 7;
const int PIN_LED   = 13;

const int SPEED = 120;             // 0-255

void setup() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_PH, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_SLEEP, HIGH);   // driver on
  analogWrite(PIN_EN, 0);          // brake
  digitalWrite(PIN_LED, HIGH);
  delay(5000);                     // time to step away
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  digitalWrite(PIN_PH, HIGH);      // forward
  analogWrite(PIN_EN, SPEED);
  delay(2000);
  analogWrite(PIN_EN, 0);          // brake
  delay(3000);

  digitalWrite(PIN_PH, LOW);       // reverse
  analogWrite(PIN_EN, SPEED);
  delay(2000);
  analogWrite(PIN_EN, 0);          // brake
  delay(3000);
}
