// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <esp_task_wdt.h>

// Forward declarations to avoid including control.h
#ifndef CONTROL_H
namespace Control {
    extern const int MOTOR_REAR_LEFT;
    extern const int MOTOR_REAR_RIGHT;
    extern const int MOTOR_FRONT_RIGHT;
    extern const int MOTOR_FRONT_LEFT;
}
#endif

extern float motors[4];  // Array to store motor values [0..1]

// Function declarations
void setupMotors();
void sendMotors();
bool motorsActive();
void testMotor(int n);
int getDutyCycle(float value);

// Helper functions
bool safeLedcAttach(uint8_t pin, double freq, uint8_t resolution);
void feedWatchdog();

// Motor pin definitions
#define MOTOR_0_PIN 12  // Rear Left
#define MOTOR_1_PIN 13  // Rear Right
#define MOTOR_2_PIN 14  // Front Right
#define MOTOR_3_PIN 15  // Front Left

// PWM Configuration
#define PWM_FREQUENCY 400  // 400Hz for better ESC compatibility
#define PWM_RESOLUTION 12  // 12-bit resolution (0-4095)
#define PWM_STOP 0         // Stop signal (0% duty cycle)
#define PWM_MIN 0          // Minimum throttle (0% duty cycle)
#define PWM_MAX 4095       // Maximum throttle (100% duty cycle for 12-bit)

// Watchdog configuration
#define WDT_TIMEOUT_MS 3000  // 3 seconds

#endif // MOTORS_H
