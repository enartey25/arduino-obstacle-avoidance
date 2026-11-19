// =============================================================================
// HARDWARE DIAGNOSTIC & CALIBRATION SKETCH
// Run this FIRST to verify all 4 ultrasonic sensors and motor polarities.
// =============================================================================

#include <Arduino.h>

// Motor Pins (L298N)
#define PIN_ENA  5
#define PIN_IN1  7
#define PIN_IN2  8
#define PIN_ENB  6
#define PIN_IN3  9
#define PIN_IN4  10

// Sensor Pins (HC-SR04)
#define PIN_TRIG_UP     2
#define PIN_ECHO_UP     12
#define PIN_TRIG_DOWN   3
#define PIN_ECHO_DOWN   13
#define PIN_TRIG_LEFT   4
#define PIN_ECHO_LEFT   A0
#define PIN_TRIG_RIGHT  11
#define PIN_ECHO_RIGHT  A1

uint16_t readDistance(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long dur = pulseIn(echo, HIGH, 18000);
    if (dur == 0) return 200;
    uint16_t d = dur / 58;
    return (d == 0) ? 200 : d;
}

void brake() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("=================================================="));
    Serial.println(F(" Autonomous Car: Hardware Diagnostics & Test     "));
    Serial.println(F(" Open Serial Monitor at 115200 baud              "));
    Serial.println(F("=================================================="));

    // Initialize Motor Pins
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    brake();

    // Initialize Sensor Pins
    pinMode(PIN_TRIG_UP, OUTPUT);
    pinMode(PIN_ECHO_UP, INPUT);
    pinMode(PIN_TRIG_DOWN, OUTPUT);
    pinMode(PIN_ECHO_DOWN, INPUT);
    pinMode(PIN_TRIG_LEFT, OUTPUT);
    pinMode(PIN_ECHO_LEFT, INPUT);
    pinMode(PIN_TRIG_RIGHT, OUTPUT);
    pinMode(PIN_ECHO_RIGHT, INPUT);

    Serial.println(F("Starting Motor Polarity Check in 2 seconds..."));
    Serial.println(F("PROP WHEELS OFF THE GROUND!"));
    delay(2000);

    // Motor 1 (Left) Test
    Serial.println(F("1. LEFT MOTOR FORWARD (500ms)..."));
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 180);
    delay(500);
    brake();
    delay(500);

    // Motor 2 (Right) Test
    Serial.println(F("2. RIGHT MOTOR FORWARD (500ms)..."));
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENB, 180);
    delay(500);
    brake();
    delay(500);

    // Both Motors Reverse Test
    Serial.println(F("3. BOTH MOTORS REVERSE (500ms)..."));
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    analogWrite(PIN_ENA, 160);
    analogWrite(PIN_ENB, 160);
    delay(500);
    brake();

    Serial.println(F("\nMotors verified. Entering live ultrasonic sensor loop...\n"));
}

void loop() {
    uint16_t d_up    = readDistance(PIN_TRIG_UP, PIN_ECHO_UP);
    delay(15);
    uint16_t d_down  = readDistance(PIN_TRIG_DOWN, PIN_ECHO_DOWN);
    delay(15);
    uint16_t d_left  = readDistance(PIN_TRIG_LEFT, PIN_ECHO_LEFT);
    delay(15);
    uint16_t d_right = readDistance(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
    delay(15);

    Serial.print(F("SENSORS (cm) | UP (Front): "));
    Serial.print(d_up);
    Serial.print(F(" | DOWN (Back): "));
    Serial.print(d_down);
    Serial.print(F(" | LEFT: "));
    Serial.print(d_left);
    Serial.print(F(" | RIGHT: "));
    Serial.println(d_right);

    delay(100);
}

