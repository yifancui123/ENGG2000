#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include <Arduino.h>
#include "Arduino.h"

// ===== TSSP58P38 multi-sensor strength reader (power-cycled) =====
// Beacon is continuous, so we blink the sensors' power. On each power-up the
// AGC is fresh and detects the beacon for a short window whose LOW-pulse
// width = strength.
//
// Wiring:
//   All TSSP VS  -> D8 (shared power pin)
//   OUT: front->A0, right->A3, back->A4, left->A5
//   All GND -> GND
//
// To add sensors: raise sensorNum and extend sensorPins in this order:
//   0=front 1=right 2=back 3=left


struct AnalogueSensor{
    AnalogueSensor(const char* a_name, int a_info_pin) 
        : name(a_name)
        ,info_pin(a_info_pin)
    {}
    const char* name;
    int info_pin;

    static constexpr int POWER_PIN              = 4;                  // shared VS for all sensors
    static constexpr int ON_MS                  = 120;   // powered ON window
    static constexpr int RESET_AGC_DURATION_MS  = 380;   // powered OFF (resets the AGC)
    unsigned long strength                      = 0;

    void setup_sensor() {
        pinMode(POWER_PIN, OUTPUT);
        digitalWrite(POWER_PIN, LOW);
        pinMode(info_pin, INPUT);
        delay(300);
    }

    // Reads the wired sensors one at a time (~50 ms each). Unwired slots stay 0.
    void read_strength() {
        strength = read_strength_once(POWER_PIN, info_pin);
    }

    static unsigned long read_strength_once(int powerPin, int outPin) {
        digitalWrite(powerPin, LOW);
        delay(RESET_AGC_DURATION_MS);                                   // reset the AGC
        digitalWrite(powerPin, HIGH);                    // fresh power-up
        auto const t0   = micros();
        // The sensor timing window.
        auto const sensor_detection_us = static_cast<unsigned long>(ON_MS * 1000UL);

        while (digitalRead(outPin) == HIGH) {            // wait for detection
            auto us_elapsed = micros() - t0;
            if (us_elapsed > sensor_detection_us){
                return 0;
            }
        }
        // After this loop, if we didnt early return, we observed a LOW signal
        // We must now find the duration of this low signal.
        unsigned long low_t0 = micros();
        while (digitalRead(outPin) == LOW) {             // wait for AGC to blind it
            auto us_elapsed = micros() - t0;
            if (us_elapsed > sensor_detection_us) {
                break;
            }
        }
        return micros() - low_t0;
    }

    static void debug_print_strength_graphs(const int n_sensors, AnalogueSensor sensors[]){
        for (int i = 0; i < n_sensors; i++) {
          if (i > 0) Serial.print("\t");
          Serial.print(sensors[i].name);
          Serial.print(":");
          Serial.print(sensors[i].strength);
        }
        Serial.println();
    }
};



// One power-cycle on ONE sensor: LOW-pulse width (us) = strength, 0 if unseen.

#endif
