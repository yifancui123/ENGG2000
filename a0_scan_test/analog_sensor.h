#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include <Arduino.h>
static const int sensorNum = 4;  // A0 front, A1 right, A2 back, A3 left (body sides)
static const int ON_MS  = 120;   // powered ON window

struct SensorFrame {
  unsigned long strength[sensorNum];  // LOW-pulse width in us, 0 = unseen
  unsigned long onStartMs;            // when this frame's ON window began
};

void setupSensor();
void sensorsUpdate();                 // call every loop(), never blocks
bool sensorsNewFrame();               // true once per finished power cycle
const SensorFrame &sensorFrame();     // the latest finished frame

#endif
