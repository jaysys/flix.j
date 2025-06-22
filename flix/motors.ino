// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Motors output control using MOSFETs
// In case of using ESCs, change PWM_STOP, PWM_MIN and PWM_MAX to appropriate values in μs, decrease PWM_FREQUENCY (to 400)

#include "motors.h"
#include "util.h"

// Motor control variables
float motors[4] = {0};

// Forward declarations
void ledcWrite(uint8_t pin, uint32_t duty);

// Motor pin definitions
#define MOTOR_0_PIN 12  // Rear Left
#define MOTOR_1_PIN 13  // Rear Right
#define MOTOR_2_PIN 14  // Front Right
#define MOTOR_3_PIN 15  // Front Left

// Array to track motor pin initialization status
bool motorPinsInitialized[4] = {false, false, false, false};

// Function to feed the watchdog during long operations
void feedWatchdog() {
    esp_task_wdt_reset();
}

// Function to safely attach a motor pin with error checking
bool safeLedcAttach(uint8_t pin, double freq, uint8_t resolution) {
    if (pin < 12 || pin > 15) {  // Only allow pins 12-15
        print("Invalid motor pin: %d\n", pin);
        return false;
    }
    
    // Check if this pin is already initialized
    if (motorPinsInitialized[pin - 12]) {
        print("Motor pin %d already initialized\n", pin);
        return true;
    }
    
    // Configure LEDC
    uint8_t channel = pin - 12;  // Use pin number as channel (0-3)
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0);  // Start with 0% duty cycle
    
    // Mark this pin as initialized
    motorPinsInitialized[pin - 12] = true;
    return true;
}

#define PWM_FREQUENCY 400  // Reduced from 1000Hz to 400Hz for better ESC compatibility
#define PWM_RESOLUTION 12
#define PWM_STOP 0
#define PWM_MIN 0
#define PWM_MAX 4095  // For 12-bit resolution (2^12 - 1)

// Motors array indexes:
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

void setupMotors() {
    print("Initializing motors...\n");
    
    // Configure pins with error checking
    print("Attaching motor pins...\n");
    
    // Initialize watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,  // 5 seconds for initialization
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
    
    // Add this task to watchdog
    esp_task_wdt_add(NULL);
    
    // Initialize each motor with error recovery
    #define INIT_MOTOR(pin, name) \
        feedWatchdog(); \
        if (!safeLedcAttach(pin, PWM_FREQUENCY, PWM_RESOLUTION)) { \
            print("Failed to initialize " name " motor\n"); \
            success = false; \
        } else { \
            print("Initialized " name " motor on pin %d\n", pin); \
        } \
        delay(50);
    
    INIT_MOTOR(MOTOR_0_PIN, "rear left");
    INIT_MOTOR(MOTOR_1_PIN, "rear right");
    INIT_MOTOR(MOTOR_2_PIN, "front right");
    INIT_MOTOR(MOTOR_3_PIN, "front left");
    
    // Reconfigure watchdog for normal operation
    twdt_config.timeout_ms = WDT_TIMEOUT_MS;
    esp_task_wdt_init(&twdt_config);
    
    // Keep the task in watchdog for normal operation
    esp_task_wdt_add(NULL);
    
    if (!success) {
        print("Warning: Some motors failed to initialize\n");
    }
    
    // Initialize motors to stopped state
    for (int i = 0; i < 4; i++) {
        motors[i] = 0;
    }
    
    sendMotors();
    
    // Re-enable watchdog timer
    #ifdef ESP32
    enableLoopWDT();  // Re-enable loop watchdog
    #endif
    
    print("Motors initialized successfully\n");
}

int getDutyCycle(float value) {
	value = constrain(value, 0, 1);
	float pwm = mapff(value, 0, 1, PWM_MIN, PWM_MAX);
	if (value == 0) pwm = PWM_STOP;
	float duty = mapff(pwm, 0, 1000000 / PWM_FREQUENCY, 0, (1 << PWM_RESOLUTION) - 1);
	return round(duty);
}

// Custom ledcWrite implementation
void ledcWrite(uint8_t pin, uint32_t duty) {
    uint8_t channel = pin - MOTOR_0_PIN;
    if (channel < 4) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel, duty);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)channel);
    }
}

void sendMotors() {
    // Only write to initialized motor pins
    if (motorPinsInitialized[0]) ledcWrite(MOTOR_0_PIN, getDutyCycle(motors[0]));
    if (motorPinsInitialized[1]) ledcWrite(MOTOR_1_PIN, getDutyCycle(motors[1]));
    if (motorPinsInitialized[2]) ledcWrite(MOTOR_2_PIN, getDutyCycle(motors[2]));
    if (motorPinsInitialized[3]) ledcWrite(MOTOR_3_PIN, getDutyCycle(motors[3]));
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n) {
	print("Testing motor %d\n", n);
	motors[n] = 1;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	print("Done\n");
}
