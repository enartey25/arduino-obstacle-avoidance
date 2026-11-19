#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// L298N Dual H-Bridge Pin Assignments
#define PIN_ENA  5   // Timer0 PWM Left
#define PIN_IN1  7   // Direction Left 1
#define PIN_IN2  8   // Direction Left 2
#define PIN_ENB  6   // Timer0 PWM Right
#define PIN_IN3  9   // Direction Right 1
#define PIN_IN4  10  // Direction Right 2

// Calibrated PWM Speed Constants (0 - 255)
#define PWM_FORWARD    210  // Cruising forward
#define PWM_BACKWARD   180  // Reversing
#define PWM_SPIN       160  // Pivot spin

void initMotors();
void setMotorSpeeds(int left_speed, int right_speed);
void executeAction(uint8_t action);
void emergencyBrake();
void emergencyReverse(uint16_t duration_ms);

#endif // MOTORS_H
