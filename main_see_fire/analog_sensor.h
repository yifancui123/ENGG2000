#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include <Arduino.h>
static const int sensorNum = 4;  // front (A0) + three more (A1, A2, A3)
static const int ON_MS  = 120;   // powered ON window
void setupSensor();
void readStrengths(unsigned long out[sensorNum]);

#endif
