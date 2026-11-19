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

// Base Calibrated Speeds
#define PWM_FAST       225  // High-speed sprint
#define PWM_SLOW       135  // Cautious glide
#define PWM_ARC_OUTER  210  // Outer wheel on sweeping curves
#define PWM_ARC_INNER  105  // Inner wheel on sweeping curves
#define PWM_SPIN       150  // Tight pivot
#define PWM_REVERSE    155  // Smooth reverse

// Slew-rate interpolation coefficient (0.0 to 1.0; 0.25 = smooth continuous glide)
#define SLEW_RATE_COEFF 0.25f

void initMotors();
void setRawMotorPWM(int left_pwm, int right_pwm);
void setTargetAction(uint8_t action, uint16_t dist_up);
void updateMotorSmoothing();
void emergencyBrake();

#endif // MOTORS_H
