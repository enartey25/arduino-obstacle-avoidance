#include "ultrasonic.h"

static const uint8_t TRIG_PINS[NUM_SENSORS] = { PIN_TRIG_UP, PIN_TRIG_DOWN, PIN_TRIG_LEFT, PIN_TRIG_RIGHT };
static const uint8_t ECHO_PINS[NUM_SENSORS] = { PIN_ECHO_UP, PIN_ECHO_DOWN, PIN_ECHO_LEFT, PIN_ECHO_RIGHT };

void initSensors() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
        digitalWrite(TRIG_PINS[i], LOW);
    }
}

uint16_t measureSingleSensor(uint8_t index) {
    if (index >= NUM_SENSORS) return 200;

    uint8_t trig = TRIG_PINS[index];
    uint8_t echo = ECHO_PINS[index];

    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long duration = pulseIn(echo, HIGH, PULSE_TIMEOUT_US);
    if (duration == 0) return 200;

    uint16_t distance_cm = duration / 58;
    if (distance_cm == 0) return 200;

    return distance_cm;
}

void readAllSensors(uint16_t out_distances[NUM_SENSORS]) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        out_distances[i] = measureSingleSensor(i);
        delay(10);  // Sequential ping delay prevents ultrasonic cross-talk
    }
}
