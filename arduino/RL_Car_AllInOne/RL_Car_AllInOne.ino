// =============================================================================
// AUTONOMOUS RL CAR - COMPLETE ALL-IN-ONE ARDUINO UNO FIRMWARE
// 4-Direction Q-Learning Policy Navigation with Hardware Safety Reflex
// Target: Arduino Uno (ATmega328P, 16 MHz) + L298N + 4x HC-SR04
// Memory: 81 bytes Flash (PROGMEM), 0 bytes SRAM
// =============================================================================

#include <Arduino.h>
#include <avr/pgmspace.h>

// Action Definitions
#define ACTION_UP     0  // Move UP (Forward)
#define ACTION_DOWN   1  // Move DOWN (Backward / Reverse)
#define ACTION_LEFT   2  // Move LEFT (Spin turn left)
#define ACTION_RIGHT  3  // Move RIGHT (Spin turn right)

// Sensor Distance Categories
#define SENSOR_BLOCKED  0  // < 15 cm
#define SENSOR_NEAR     1  // 15 - 35 cm
#define SENSOR_CLEAR    2  // > 35 cm

#define DIST_BLOCKED_THRESH_CM  15
#define DIST_NEAR_THRESH_CM     35
#define DIST_EMERGENCY_STOP_CM  10

// L298N Motor Driver Pins
#define PIN_ENA  5   // Timer0 PWM Left
#define PIN_IN1  7   // Direction Left 1
#define PIN_IN2  8   // Direction Left 2
#define PIN_ENB  6   // Timer0 PWM Right
#define PIN_IN3  9   // Direction Right 1
#define PIN_IN4  10  // Direction Right 2

// Motor Speeds (0 - 255 PWM)
#define PWM_FORWARD    210
#define PWM_BACKWARD   180
#define PWM_SPIN       160

// 4 Ultrasonic Sensor Pins (HC-SR04)
#define PIN_TRIG_UP     2
#define PIN_ECHO_UP     12
#define PIN_TRIG_DOWN   3
#define PIN_ECHO_DOWN   13
#define PIN_TRIG_LEFT   4
#define PIN_ECHO_LEFT   A0
#define PIN_TRIG_RIGHT  11
#define PIN_ECHO_RIGHT  A1

#define PULSE_TIMEOUT_US 18000
#define TELEMETRY_INTERVAL_MS 150

// 81-Byte Policy Array in Flash Memory (PROGMEM)
const uint8_t PROGMEM Q_POLICY[81] = {
    3, 3, 3, 3, 3, 3, 2, 2, 3,  // States 0..8
    3, 3, 3, 3, 3, 1, 2, 1, 3,  // States 9..17
    1, 1, 2, 1, 1, 1, 2, 2, 1,  // States 18..26
    3, 3, 3, 3, 3, 3, 2, 2, 2,  // States 27..35
    3, 3, 3, 3, 2, 3, 1, 3, 3,  // States 36..44
    1, 1, 3, 2, 3, 3, 2, 3, 2,  // States 45..53
    0, 0, 0, 0, 0, 0, 0, 0, 0,  // States 54..62
    0, 0, 3, 0, 0, 0, 2, 0, 0,  // States 63..71
    1, 3, 1, 2, 1, 1, 1, 0, 0   // States 72..80
};

const char* ACTION_TEXT[] = {
    "UP (FORWARD)",
    "DOWN (BACKWARD)",
    "LEFT",
    "RIGHT"
};

static uint16_t distances[4];
static unsigned long last_telemetry_time = 0;

// Motor Control Helpers
void brake() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
}

void setMotorSpeeds(int left_speed, int right_speed) {
    if (left_speed > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, constrain(left_speed, 0, 255));
    } else if (left_speed < 0) {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, constrain(-left_speed, 0, 255));
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, 0);
    }

    if (right_speed > 0) {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, constrain(right_speed, 0, 255));
    } else if (right_speed < 0) {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENB, constrain(-right_speed, 0, 255));
    } else {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, 0);
    }
}

void executeAction(uint8_t action) {
    switch (action) {
        case ACTION_UP:
            setMotorSpeeds(PWM_FORWARD, PWM_FORWARD);
            break;
        case ACTION_DOWN:
            setMotorSpeeds(-PWM_BACKWARD, -PWM_BACKWARD);
            break;
        case ACTION_LEFT:
            setMotorSpeeds(-PWM_SPIN, PWM_SPIN);
            break;
        case ACTION_RIGHT:
            setMotorSpeeds(PWM_SPIN, -PWM_SPIN);
            break;
        default:
            brake();
            break;
    }
}

// Ultrasonic Sensor Helpers
uint16_t measureSensor(uint8_t trig, uint8_t echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long duration = pulseIn(echo, HIGH, PULSE_TIMEOUT_US);
    if (duration == 0) return 200;
    uint16_t dist = duration / 58;
    return (dist == 0) ? 200 : dist;
}

void readAllSensors() {
    distances[0] = measureSensor(PIN_TRIG_UP, PIN_ECHO_UP);
    delay(10);
    distances[1] = measureSensor(PIN_TRIG_DOWN, PIN_ECHO_DOWN);
    delay(10);
    distances[2] = measureSensor(PIN_TRIG_LEFT, PIN_ECHO_LEFT);
    delay(10);
    distances[3] = measureSensor(PIN_TRIG_RIGHT, PIN_ECHO_RIGHT);
    delay(10);
}

// State Discretizer and Encoder
uint8_t discretize(uint16_t dist_cm) {
    if (dist_cm < DIST_BLOCKED_THRESH_CM) return SENSOR_BLOCKED;
    if (dist_cm <= DIST_NEAR_THRESH_CM)    return SENSOR_NEAR;
    return SENSOR_CLEAR;
}

uint8_t encodeState(uint8_t s_up, uint8_t s_down, uint8_t s_left, uint8_t s_right) {
    return (s_up * 27) + (s_down * 9) + (s_left * 3) + s_right;
}

uint8_t getPolicyAction(uint8_t state_id) {
    if (state_id >= 81) return ACTION_RIGHT;
    return pgm_read_byte(&(Q_POLICY[state_id]));
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("=================================================="));
    Serial.println(F(" Autonomous RL Car - Single File Firmware         "));
    Serial.println(F(" Flash Policy: 81 bytes | RAM: 0 bytes            "));
    Serial.println(F("=================================================="));

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    brake();

    pinMode(PIN_TRIG_UP, OUTPUT);
    pinMode(PIN_ECHO_UP, INPUT);
    pinMode(PIN_TRIG_DOWN, OUTPUT);
    pinMode(PIN_ECHO_DOWN, INPUT);
    pinMode(PIN_TRIG_LEFT, OUTPUT);
    pinMode(PIN_ECHO_LEFT, INPUT);
    pinMode(PIN_TRIG_RIGHT, OUTPUT);
    pinMode(PIN_ECHO_RIGHT, INPUT);

    // Initial check and brief motor pulse
    readAllSensors();
    Serial.print(F("Initial distances: UP="));
    Serial.print(distances[0]);
    Serial.print(F(" DOWN="));
    Serial.print(distances[1]);
    Serial.print(F(" LEFT="));
    Serial.print(distances[2]);
    Serial.print(F(" RIGHT="));
    Serial.println(distances[3]);

    Serial.println(F("Motor pulse test (400ms)..."));
    setMotorSpeeds(PWM_FORWARD, PWM_FORWARD);
    delay(400);
    brake();

    Serial.println(F("Autonomous mode begins in 2 seconds..."));
    delay(2000);
}

void loop() {
    // 1. Measure all 4 sensors sequentially
    readAllSensors();
    uint16_t d_up    = distances[0];
    uint16_t d_down  = distances[1];
    uint16_t d_left  = distances[2];
    uint16_t d_right = distances[3];

    // 2. Hardware Safety Override
    if (d_up < DIST_EMERGENCY_STOP_CM) {
        Serial.println(F("[SAFETY OVERRIDE] Critical frontal proximity (<10cm)!"));
        brake();
        if (d_down > DIST_BLOCKED_THRESH_CM) {
            setMotorSpeeds(-PWM_BACKWARD, -PWM_BACKWARD);
            delay(250);
        } else {
            setMotorSpeeds((d_left > d_right) ? -PWM_SPIN : PWM_SPIN,
                           (d_left > d_right) ?  PWM_SPIN : -PWM_SPIN);
            delay(300);
        }
        brake();
        return;
    }

    // 3. Discretize distances (0=Blocked, 1=Near, 2=Clear)
    uint8_t s_up    = discretize(d_up);
    uint8_t s_down  = discretize(d_down);
    uint8_t s_left  = discretize(d_left);
    uint8_t s_right = discretize(d_right);

    // 4. Encode state index
    uint8_t state_id = encodeState(s_up, s_down, s_left, s_right);

    // 5. Flash policy query
    uint8_t action = getPolicyAction(state_id);

    // 6. Motor drive
    executeAction(action);

    // 7. Live Serial Telemetry
    unsigned long now = millis();
    if (now - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_time = now;
        Serial.print(F("[RL] Dist:[U:"));
        Serial.print(d_up);
        Serial.print(F(", D:"));
        Serial.print(d_down);
        Serial.print(F(", L:"));
        Serial.print(d_left);
        Serial.print(F(", R:"));
        Serial.print(d_right);
        Serial.print(F("] -> St:"));
        Serial.print(state_id);
        Serial.print(F(" -> Act:"));
        Serial.println(ACTION_TEXT[action]);
    }
}

