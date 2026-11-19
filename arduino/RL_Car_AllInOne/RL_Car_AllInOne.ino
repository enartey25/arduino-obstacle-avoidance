// =============================================================================
// AUTONOMOUS RL CAR - COMPLETE ALL-IN-ONE ARDUINO UNO FIRMWARE
// Continuous-Like Smooth Motion with 7 Actions & Variable Motor Speeds
// Memory: 81 bytes Flash (PROGMEM), 0 bytes SRAM
// =============================================================================

#include <Arduino.h>
#include <avr/pgmspace.h>

// 7 Smooth Action Definitions
#define ACTION_FORWARD_FAST 0  // Fast forward sprint in open space
#define ACTION_FORWARD_SLOW 1  // Cautious forward glide near obstacles
#define ACTION_CURVE_LEFT   2  // Smooth sweeping left arc (continuous turn)
#define ACTION_CURVE_RIGHT  3  // Smooth sweeping right arc (continuous turn)
#define ACTION_SPIN_LEFT    4  // In-place pivot left (for tight obstacles)
#define ACTION_SPIN_RIGHT   5  // In-place pivot right (for tight obstacles)
#define ACTION_REVERSE      6  // Smooth reverse

// Sensor Distance Categories
#define SENSOR_BLOCKED  0  // < 18 cm
#define SENSOR_NEAR     1  // 18 - 40 cm
#define SENSOR_CLEAR    2  // > 40 cm

#define DIST_BLOCKED_THRESH_CM  18
#define DIST_NEAR_THRESH_CM     40
#define DIST_EMERGENCY_STOP_CM  12

// L298N Motor Driver Pins
#define PIN_ENA  5   // Timer0 PWM Left
#define PIN_IN1  7   // Direction Left 1
#define PIN_IN2  8   // Direction Left 2
#define PIN_ENB  6   // Timer0 PWM Right
#define PIN_IN3  9   // Direction Right 1
#define PIN_IN4  10  // Direction Right 2

// Base Calibrated Speeds
#define PWM_FAST       225
#define PWM_SLOW       135
#define PWM_ARC_OUTER  210
#define PWM_ARC_INNER  105
#define PWM_SPIN       150
#define PWM_REVERSE    155

#define SLEW_RATE_COEFF 0.25f

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
    5, 5, 5, 4, 5, 5, 4, 4, 1,  // States 0..8
    5, 5, 4, 5, 5, 4, 6, 5, 4,  // States 9..17
    6, 6, 6, 6, 6, 6, 4, 6, 6,  // States 18..26
    5, 5, 5, 5, 5, 5, 4, 4, 4,  // States 27..35
    5, 5, 4, 5, 4, 5, 4, 6, 4,  // States 36..44
    3, 6, 4, 3, 3, 3, 2, 4, 3,  // States 45..53
    0, 0, 0, 0, 0, 0, 0, 0, 0,  // States 54..62
    0, 0, 3, 0, 0, 0, 0, 0, 0,  // States 63..71
    3, 3, 3, 3, 3, 3, 2, 3, 0   // States 72..80
};

const char* ACTION_TEXT[] = {
    "FORWARD_FAST",
    "FORWARD_SLOW",
    "CURVE_LEFT",
    "CURVE_RIGHT",
    "SPIN_LEFT",
    "SPIN_RIGHT",
    "REVERSE"
};

static uint16_t distances[4];
static unsigned long last_telemetry_time = 0;

static float target_left_speed = 0.0f;
static float target_right_speed = 0.0f;
static float current_left_speed = 0.0f;
static float current_right_speed = 0.0f;

// Motor Control Helpers
void setRawMotorPWM(int left_pwm, int right_pwm) {
    if (left_pwm > 0) {
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, constrain(left_pwm, 0, 255));
    } else if (left_pwm < 0) {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, constrain(-left_pwm, 0, 255));
    } else {
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, 0);
    }

    if (right_pwm > 0) {
        digitalWrite(PIN_IN3, HIGH);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, constrain(right_pwm, 0, 255));
    } else if (right_pwm < 0) {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, HIGH);
        analogWrite(PIN_ENB, constrain(-right_pwm, 0, 255));
    } else {
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        analogWrite(PIN_ENB, 0);
    }
}

void brake() {
    target_left_speed   = 0.0f;
    target_right_speed  = 0.0f;
    current_left_speed  = 0.0f;
    current_right_speed = 0.0f;
    setRawMotorPWM(0, 0);
}

void setTargetAction(uint8_t action, uint16_t dist_up) {
    // Proportional Continuous Speed Scaling
    float speed_factor = 1.0f;
    if (dist_up > 70) {
        speed_factor = 1.10f; // Boost speed in open field
    } else if (dist_up < 40 && dist_up > 18) {
        speed_factor = 0.85f; // Automatically ease throttle down as obstacle nears
    }

    switch (action) {
        case ACTION_FORWARD_FAST:
            target_left_speed  = PWM_FAST * speed_factor;
            target_right_speed = PWM_FAST * speed_factor;
            break;

        case ACTION_FORWARD_SLOW:
            target_left_speed  = PWM_SLOW;
            target_right_speed = PWM_SLOW;
            break;

        case ACTION_CURVE_LEFT:
            target_left_speed  = PWM_ARC_INNER;
            target_right_speed = PWM_ARC_OUTER * speed_factor;
            break;

        case ACTION_CURVE_RIGHT:
            target_left_speed  = PWM_ARC_OUTER * speed_factor;
            target_right_speed = PWM_ARC_INNER;
            break;

        case ACTION_SPIN_LEFT:
            target_left_speed  = -PWM_SPIN;
            target_right_speed =  PWM_SPIN;
            break;

        case ACTION_SPIN_RIGHT:
            target_left_speed  =  PWM_SPIN;
            target_right_speed = -PWM_SPIN;
            break;

        case ACTION_REVERSE:
            target_left_speed  = -PWM_REVERSE;
            target_right_speed = -PWM_REVERSE;
            break;

        default:
            target_left_speed  = 0.0f;
            target_right_speed = 0.0f;
            break;
    }
}

void updateMotorSmoothing() {
    current_left_speed  += (target_left_speed  - current_left_speed)  * SLEW_RATE_COEFF;
    current_right_speed += (target_right_speed - current_right_speed) * SLEW_RATE_COEFF;
    setRawMotorPWM((int)current_left_speed, (int)current_right_speed);
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

uint8_t discretize(uint16_t dist_cm) {
    if (dist_cm < DIST_BLOCKED_THRESH_CM) return SENSOR_BLOCKED;
    if (dist_cm <= DIST_NEAR_THRESH_CM)    return SENSOR_NEAR;
    return SENSOR_CLEAR;
}

uint8_t encodeState(uint8_t s_up, uint8_t s_down, uint8_t s_left, uint8_t s_right) {
    return (s_up * 27) + (s_down * 9) + (s_left * 3) + s_right;
}

uint8_t getPolicyAction(uint8_t state_id) {
    if (state_id >= 81) return ACTION_SPIN_RIGHT;
    return pgm_read_byte(&(Q_POLICY[state_id]));
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("=================================================="));
    Serial.println(F(" Autonomous RL Car - Continuous-Like Navigation   "));
    Serial.println(F(" Variable Speeds & Slew-Rate Acceleration Filter  "));
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

    readAllSensors();
    Serial.print(F("Initial distances: UP="));
    Serial.print(distances[0]);
    Serial.print(F(" DOWN="));
    Serial.print(distances[1]);
    Serial.print(F(" LEFT="));
    Serial.print(distances[2]);
    Serial.print(F(" RIGHT="));
    Serial.println(distances[3]);

    Serial.println(F("Testing motors: Smooth forward ramp (400ms)..."));
    setTargetAction(ACTION_FORWARD_SLOW, 100);
    for (int i = 0; i < 20; i++) {
        updateMotorSmoothing();
        delay(20);
    }
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

    // 2. Hardware Safety Override (<12cm)
    if (d_up < DIST_EMERGENCY_STOP_CM) {
        Serial.println(F("[SAFETY OVERRIDE] Critical frontal proximity (<12cm)!"));
        brake();
        delay(30);

        if (d_down > DIST_BLOCKED_THRESH_CM) {
            setTargetAction(ACTION_REVERSE, d_up);
            for (int i = 0; i < 12; i++) {
                updateMotorSmoothing();
                delay(20);
            }
        } else {
            uint8_t spin_act = (d_left > d_right) ? ACTION_SPIN_LEFT : ACTION_SPIN_RIGHT;
            setTargetAction(spin_act, d_up);
            for (int i = 0; i < 15; i++) {
                updateMotorSmoothing();
                delay(20);
            }
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

    // 6. Set target motor speeds with dynamic proportional scaling
    setTargetAction(action, d_up);

    // Smooth motor update loop
    for (int k = 0; k < 5; k++) {
        updateMotorSmoothing();
        delay(10);
    }

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
