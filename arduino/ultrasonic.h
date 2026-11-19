#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

#define NUM_SENSORS 4

// 4 Cardinal Direction HC-SR04 Pins:
// Sensor 0: UP (Front)
#define PIN_TRIG_UP     2
#define PIN_ECHO_UP     12

// Sensor 1: DOWN (Back/Rear)
#define PIN_TRIG_DOWN   3
#define PIN_ECHO_DOWN   13

// Sensor 2: LEFT
#define PIN_TRIG_LEFT   4
#define PIN_ECHO_LEFT   A0   // Analog A0 used as digital pin

// Sensor 3: RIGHT
#define PIN_TRIG_RIGHT  11
#define PIN_ECHO_RIGHT  A1   // Analog A1 used as digital pin

// Pulse timeout in microseconds (~300 cm max range)
#define PULSE_TIMEOUT_US  18000

void initSensors();
uint16_t measureSingleSensor(uint8_t index);
void readAllSensors(uint16_t out_distances[NUM_SENSORS]);

#endif // ULTRASONIC_H
