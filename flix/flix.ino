// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"

#define SERIAL_BAUDRATE 115200
#define WIFI_ENABLED 1

// Function tracing
#define ENABLE_FUNCTION_TRACING 1

#if ENABLE_FUNCTION_TRACING
struct FunctionTrace {
    const char* name;
    unsigned long startTime;
    unsigned long maxTime;
    unsigned long totalTime;
    unsigned long callCount;
};

FunctionTrace functionTraces[] = {
    {"readIMU", 0, 0, 0, 0},
    {"step", 0, 0, 0, 0},
    {"readRC", 0, 0, 0, 0},
    {"estimate", 0, 0, 0, 0},
    {"control", 0, 0, 0, 0},
    {"sendMotors", 0, 0, 0, 0},
    {"handleInput", 0, 0, 0, 0},
    #if WIFI_ENABLED
    {"processMavlink", 0, 0, 0, 0},
    #endif
    {"logData", 0, 0, 0, 0}
};

#define FUNCTION_TRACE_START(func) \
    unsigned long _start_##func = micros();

#define FUNCTION_TRACE_END(func) \
    do { \
        unsigned long _duration = micros() - _start_##func; \
        for (auto& trace : functionTraces) { \
            if (strcmp(trace.name, #func) == 0) { \
                trace.callCount++; \
                trace.totalTime += _duration; \
                if (_duration > trace.maxTime) trace.maxTime = _duration; \
                break; \
            } \
        } \
    } while(0);

void printFunctionStats() {
    static unsigned long lastPrint = 0;
    unsigned long now = millis();
    
    if (now - lastPrint >= 1000) { // Print every second
        lastPrint = now;
        
        print("\n--- Function Statistics (us) ---\n");
        print("Function        |   Avg |   Max |  Calls\n");
        print("----------------|-------|-------|-------\n");
        
        for (const auto& trace : functionTraces) {
            if (trace.callCount > 0) {
                print("%-15s|%6lu |%6lu |%6lu\n", 
                    trace.name, 
                    trace.totalTime / trace.callCount, 
                    trace.maxTime,
                    trace.callCount);
            }
        }
        
        // Reset counters
        for (auto& trace : functionTraces) {
            trace.callCount = 0;
            trace.totalTime = 0;
            trace.maxTime = 0;
        }
    }
}
#else
#define FUNCTION_TRACE_START(func)
#define FUNCTION_TRACE_END(func)
inline void printFunctionStats() {}
#endif

double t = NAN; // current step time, s
float dt; // time delta from previous step, s
int16_t channels[16]; // raw rc channels
float controls[16]; // normalized controls in range [-1..1] ([0..1] for throttle)
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]

void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	print("Initializing flix\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	setupRC();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
    // Function tracing
    FUNCTION_TRACE_START(readIMU);
    readIMU();
    FUNCTION_TRACE_END(readIMU);
    
    FUNCTION_TRACE_START(step);
    step();
    FUNCTION_TRACE_END(step);
    
    FUNCTION_TRACE_START(readRC);
    readRC();
    FUNCTION_TRACE_END(readRC);
    
    FUNCTION_TRACE_START(estimate);
    estimate();
    FUNCTION_TRACE_END(estimate);
    
    FUNCTION_TRACE_START(control);
    control();
    FUNCTION_TRACE_END(control);
    
    FUNCTION_TRACE_START(sendMotors);
    sendMotors();
    FUNCTION_TRACE_END(sendMotors);
    
    FUNCTION_TRACE_START(handleInput);
    handleInput();
    FUNCTION_TRACE_END(handleInput);
    
#if WIFI_ENABLED
    FUNCTION_TRACE_START(processMavlink);
    processMavlink();
    FUNCTION_TRACE_END(processMavlink);
#endif
    
    FUNCTION_TRACE_START(logData);
    logData();
    FUNCTION_TRACE_END(logData);
    
    syncParameters();
    
    // Print function statistics every second
    printFunctionStats();
}
