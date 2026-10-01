#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include <Arduino.h>
static const int sensorNum = 1;                  // how many sensors are wired (1-4)
static const int ON_MS  = 25;   // powered ON window
void setupSensor();
void readStrengths(unsigned long out[4]);

#endif