// =============================================================================
// AUTONOMOUS RL CAR - CONTINUOUS-LIKE SMOOTH ARDUINO UNO FIRMWARE
// 7-Action Policy with Proportional Speed Scaling & Slew-Rate Smoothing
// Memory: 81 bytes Flash (PROGMEM), 0 bytes SRAM
// =============================================================================

#include <Arduino.h>
#include "policy.h"
#include "motors.h"
#include "ultrasonic.h"

#define TELEMETRY_INTERVAL_MS 150
#define MOTOR_SMOOTH_INTERVAL_MS 20

static uint16_t distances[NUM_SENSORS];
static unsigned long last_telemetry_time = 0;
static unsigned long last_smooth_time = 0;

const char* ACTION_TEXT[] = {
    "FORWARD_FAST",
    "FORWARD_SLOW",
    "CURVE_LEFT",
    "CURVE_RIGHT",
    "SPIN_LEFT",
    "SPIN_RIGHT",
    "REVERSE"
};

void runHardwareSelfTest() {
    Serial.println(F("--- Running Hardware Self-Test ---"));
    readAllSensors(distances);
    Serial.print(F("Initial Distances (cm): UP="));
    Serial.print(distances[0]);
    Serial.print(F(", DOWN="));
    Serial.print(distances[1]);
    Serial.print(F(", LEFT="));
    Serial.print(distances[2]);
    Serial.print(F(", RIGHT="));
    Serial.println(distances[3]);

    Serial.println(F("Testing motors: Smooth forward ramp (400ms)..."));
    setTargetAction(ACTION_FORWARD_SLOW, 100);
    for (int i = 0; i < 20; i++) {
        updateMotorSmoothing();
        delay(20);
    }
    emergencyBrake();

    Serial.println(F("Autonomous mode begins in 2 seconds..."));
    delay(2000);
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("=================================================="));
    Serial.println(F(" Autonomous RL Car - Continuous-Like Navigation   "));
    Serial.println(F(" Features: Variable Speeds & Slew-Rate Smoothing  "));
    Serial.println(F(" Flash Policy: 81 bytes | RAM: 0 bytes            "));
    Serial.println(F("=================================================="));

    initMotors();
    initSensors();

    runHardwareSelfTest();
}

void loop() {
    // 1. Sequential Ultrasonic Measurement: [UP, DOWN, LEFT, RIGHT]
    readAllSensors(distances);
    uint16_t d_up    = distances[0];
    uint16_t d_down  = distances[1];
    uint16_t d_left  = distances[2];
    uint16_t d_right = distances[3];

    // 2. INDEPENDENT HARDWARE SAFETY OVERRIDE (<10cm)
    if (d_up < DIST_EMERGENCY_STOP_CM) {
        Serial.println(F("[SAFETY OVERRIDE] Critical frontal proximity (<10cm)!"));
        emergencyBrake();
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
        emergencyBrake();
        return;
    }

    // 3. Discretize distances into categories: 0=Blocked, 1=Near, 2=Clear
    uint8_t s_up    = discretize_distance(d_up);
    uint8_t s_down  = discretize_distance(d_down);
    uint8_t s_left  = discretize_distance(d_left);
    uint8_t s_right = discretize_distance(d_right);

    // 4. Encode state ID: (s_up*27) + (s_down*9) + (s_left*3) + s_right (0..80)
    uint8_t state_id = encode_state(s_up, s_down, s_left, s_right);

    // 5. Query optimal action from Flash Memory (PROGMEM)
    uint8_t action = get_policy_action(state_id);

    // 6. Set target motor speeds with dynamic proportional speed scaling
    setTargetAction(action, d_up);

    // High frequency motor smoothing execution (smooth continuous gliding)
    for (int k = 0; k < 5; k++) {
        updateMotorSmoothing();
        delay(10);
    }

    // 7. Non-blocking Serial Telemetry
    unsigned long current_time = millis();
    if (current_time - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_time = current_time;
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
