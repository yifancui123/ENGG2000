#include <Arduino.h>
#include "ir_sensor.h"

void testDirectionalSensors() {
  const int sensorPins[] = {A3, A4, A5};
  const char* directions[] = {"LEFT", "RIGHT", "BACK"};

  for (int i = 0; i < 3; i++) {
    if (sawBeacon(sensorPins[i])) {
      Serial.print("Beacon detected: ");
      Serial.println(directions[i]);
    }
  }
}