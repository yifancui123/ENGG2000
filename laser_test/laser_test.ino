// laser_test: turn the laser on and keep it on.
// Laser on D12 (same pin as main_see_fire).
// Serial 115200: type 0 = off, 1 = on.

const int LASER_PIN = 12;

void setup() {
  Serial.begin(115200);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);
  Serial.println("laser_test: laser ON (D12). Type 0 = off, 1 = on.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '0') {
      digitalWrite(LASER_PIN, LOW);
      Serial.println("laser OFF");
    } else if (c == '1') {
      digitalWrite(LASER_PIN, HIGH);
      Serial.println("laser ON");
    }
  }
}
