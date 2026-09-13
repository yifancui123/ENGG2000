// ===== MQ Sentinel — Single TSSP58P38 Strength Test =====
// Quick bench test for ONE TSSP58P38 sensor (test only).
// Strength = LOW pulse width in microseconds (wider = stronger / more on-axis).
// Timed with pulseIn(), NOT analogRead(): OUT is a digital pin, so the
// strength lives in TIME, not voltage.

// Single sensor. OUT -> D12 (same wiring as tssp_continuous_test).
const int SENSOR_PIN = 12;

void setup() {
  Serial.begin(115200);
  pinMode(SENSOR_PIN, INPUT);   // push-pull output, no pull-up needed
}

void loop() {
  // Measure how long OUT stays LOW = signal strength.
  // 30 ms timeout so a silent sensor returns 0 instead of hanging.
  unsigned long strength = pulseIn(SENSOR_PIN, LOW, 30000UL);

  Serial.print("Strength:");
  Serial.println(strength);

  delay(20);
}
