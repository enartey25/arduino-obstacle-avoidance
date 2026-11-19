#include "motors.h"
#include "policy.h"

static float target_left_speed = 0.0f;
static float target_right_speed = 0.0f;
static float current_left_speed = 0.0f;
static float current_right_speed = 0.0f;

void initMotors() {
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    emergencyBrake();
}

void setRawMotorPWM(int left_pwm, int right_pwm) {
    // Left motor direction & PWM
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

    // Right motor direction & PWM
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

void setTargetAction(uint8_t action, uint16_t dist_up) {
    // Continuous speed scaling: proportionally scale speed based on forward distance
    float speed_factor = 1.0f;
    if (dist_up > 70) {
        speed_factor = 1.10f; // Boost speed in wide open clear field
    } else if (dist_up < 35 && dist_up > 15) {
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
            // Continuous sweeping arc: both wheels move forward, outer wheel faster
            target_left_speed  = PWM_ARC_INNER;
            target_right_speed = PWM_ARC_OUTER * speed_factor;
            break;

        case ACTION_CURVE_RIGHT:
            // Continuous sweeping arc: both wheels move forward, outer wheel faster
            target_left_speed  = PWM_ARC_OUTER * speed_factor;
            target_right_speed = PWM_ARC_INNER;
            break;

        case ACTION_SPIN_LEFT:
            // In-place pivot for tight obstacles
            target_left_speed  = -PWM_SPIN;
            target_right_speed =  PWM_SPIN;
            break;

        case ACTION_SPIN_RIGHT:
            // In-place pivot for tight obstacles
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
    // Exponential slew-rate interpolation eliminates all motor jerk
    current_left_speed  += (target_left_speed  - current_left_speed)  * SLEW_RATE_COEFF;
    current_right_speed += (target_right_speed - current_right_speed) * SLEW_RATE_COEFF;

    setRawMotorPWM((int)current_left_speed, (int)current_right_speed);
}

void emergencyBrake() {
    target_left_speed   = 0.0f;
    target_right_speed  = 0.0f;
    current_left_speed  = 0.0f;
    current_right_speed = 0.0f;
    setRawMotorPWM(0, 0);
}
