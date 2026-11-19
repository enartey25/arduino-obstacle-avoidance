#include "motors.h"
#include "policy.h"

void initMotors() {
    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_ENB, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    emergencyBrake();
}

void setMotorSpeeds(int left_speed, int right_speed) {
    // Left motor
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

    // Right motor
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
        case ACTION_UP:      // Move UP (Forward)
            setMotorSpeeds(PWM_FORWARD, PWM_FORWARD);
            break;

        case ACTION_DOWN:    // Move DOWN (Backward / Reverse)
            setMotorSpeeds(-PWM_BACKWARD, -PWM_BACKWARD);
            break;

        case ACTION_LEFT:    // Move LEFT (Spin turn left)
            setMotorSpeeds(-PWM_SPIN, PWM_SPIN);
            break;

        case ACTION_RIGHT:   // Move RIGHT (Spin turn right)
            setMotorSpeeds(PWM_SPIN, -PWM_SPIN);
            break;

        default:
            emergencyBrake();
            break;
    }
}

void emergencyBrake() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
}

void emergencyReverse(uint16_t duration_ms) {
    setMotorSpeeds(-PWM_BACKWARD, -PWM_BACKWARD);
    delay(duration_ms);
    emergencyBrake();
}
