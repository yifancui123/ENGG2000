// ===== TSSP58P38 Multi-Sensor Strength Reader — power-cycled at 2 Hz =====
// Combines tssp_strength (4 sensors, Serial Plotter output) with the
// tssp_reader power-cycle trick (the beacon is continuous, so we blink the
// sensors' power). On each power-up the AGC is fresh and detects the beacon
// for a short window whose LOW-pulse width = strength.
//
// Wiring:
//   All 4 TSSP VS  -> D7   (shared power pin; ~1.8 mA total, OK for a Nano pin)
//   OUT: front->A2, right->A3, back->A4, left->A5
//   All GND -> GND
//
// View in Serial Plotter (Tools -> Serial Plotter) at 115200 to see all four.

const int POWER_PIN = 8;                  // shared VS for all 4 sensors
int sensorPins[4] = {A0, A3, A4, A5};     // 0=front 1=right 2=back 3=left

// ---- power-cycle timing: on/off twice per second (2 Hz) ----
const int OFF_MS = 25;   // powered OFF 250 ms (resets the AGC)
const int ON_MS  = 25;   // powered ON  up to 250 ms  (250 + 250 = 2 Hz)

// One power-cycle on ONE sensor: LOW-pulse width (us) = strength, 0 if unseen.
unsigned long readStrengthOnce(int powerPin, int outPin) {
  digitalWrite(powerPin, LOW);
  delay(OFF_MS);                                   // reset the AGC
  digitalWrite(powerPin, HIGH);                    // fresh power-up
  unsigned long tStart   = micros();
  unsigned long windowUs = (unsigned long)ON_MS * 1000UL;

  while (digitalRead(outPin) == HIGH) {            // wait for detection
    if (micros() - tStart > windowUs) return 0;
  }
  unsigned long lowStart = micros();
  while (digitalRead(outPin) == LOW) {             // wait for AGC to blind it
    if (micros() - tStart > windowUs) break;
  }
  return micros() - lowStart;
}

void setup() {
  Serial.begin(115200);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);
  for (int i = 0; i < 4; i++) pinMode(sensorPins[i], INPUT);
  delay(300);
}

void loop() {
  // Reads the 4 sensors one at a time (4 power-cycles per line, ~2 s/update).
  unsigned long front = readStrengthOnce(POWER_PIN, sensorPins[0]);
  unsigned long right = readStrengthOnce(POWER_PIN, sensorPins[1]);
  unsigned long back  = readStrengthOnce(POWER_PIN, sensorPins[2]);
  unsigned long left  = readStrengthOnce(POWER_PIN, sensorPins[3]);

  Serial.print("Front:");   Serial.print(front);
  Serial.print("\tRight:"); Serial.print(right);
  Serial.print("\tBack:");  Serial.print(back);
  Serial.print("\tLeft:");  Serial.println(left);
}