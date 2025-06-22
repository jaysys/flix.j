// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include <SPI.h>
#include "MPU9250_WE.h"
#include "lpf.h"
#include "util.h"

// For ESP32 watchdog timer
#ifdef ESP32
#include "esp_task_wdt.h"
#endif

#define CS_PIN 10
#define IMU_RESET_PIN 9  // Connect this to the IMU's reset pin if available

MPU9250_WE IMU = MPU9250_WE(&SPI, CS_PIN, true); // true for SPI mode

void resetIMU() {
    #ifdef IMU_RESET_PIN
    pinMode(IMU_RESET_PIN, OUTPUT);
    digitalWrite(IMU_RESET_PIN, LOW);
    delay(10);
    digitalWrite(IMU_RESET_PIN, HIGH);
    delay(100);  // Give some time to reset
    #endif
}

void disableWatchdog() {
#ifdef ESP32
    disableCore0WDT();
    disableCore1WDT();
    disableLoopWDT();
#endif
}

void enableWatchdog() {
#ifdef ESP32
    enableLoopWDT();
#endif
}

Vector accBias;
Vector accScale(1, 1, 1);
Vector gyroBias;

void setupIMU() {
    print("Starting IMU initialization...\n");
    disableWatchdog();  // Disable watchdog during initialization
    
    // Give some time for the SPI bus to stabilize
    delay(100);
    
    // Reset IMU hardware
    resetIMU();
    
    // Initialize the IMU with retry logic
    bool initSuccess = false;
    for (int i = 0; i < 3; i++) {  // Try up to 3 times
        print("MPU9250 init attempt %d...", i + 1);
        bool result = IMU.init();
        print("%s\n", result ? "OK" : "FAILED");
        
        if (result) {
            // Check WHO_AM_I register
            byte whoAmI = IMU.whoAmI();
            print("WHO_AM_I: 0x%02X\n", whoAmI);
            
            if (whoAmI == 0x71 || whoAmI == 0x73) {  // 0x71 or 0x73 are valid MPU9250/6500 IDs
                initSuccess = true;
                break;
            } else {
                print("Invalid WHO_AM_I value\n");
            }
        }
        delay(100);
    }
    
    if (!initSuccess) {
        print("ERROR: MPU9250 init failed after multiple attempts\n");
        enableWatchdog();
        return;
    }
    print("MPU9250 initialized successfully\n");
    
    // Configure IMU settings
    print("Configuring IMU...\n");
    configureIMU();
    
    // Verify IMU is responding
    if (IMU.whoAmI() != 0x71) {
        print("ERROR: IMU not responding correctly\n");
        enableWatchdog();
        return;
    }
    
    enableWatchdog();  // Re-enable watchdog after initialization
    print("IMU setup complete\n");
}

void configureIMU() {
    // Basic configuration based on MPU9250_WE example
    IMU.autoOffsets();
    
    // Set accelerometer range to ±4g
    // Options: MPU9250_ACC_RANGE_2G, MPU9250_ACC_RANGE_4G, 
    //          MPU9250_ACC_RANGE_8G, MPU9250_ACC_RANGE_16G
    IMU.setAccRange(MPU9250_ACC_RANGE_4G);
    
    // Set gyroscope range to ±2000°/s
    // Options: MPU9250_GYRO_RANGE_250, MPU9250_GYRO_RANGE_500,
    //          MPU9250_GYRO_RANGE_1000, MPU9250_GYRO_RANGE_2000
    IMU.setGyrRange(MPU9250_GYRO_RANGE_2000);
    
    // Enable digital low pass filter for gyroscope
    IMU.enableGyrDLPF();
    
    // Set digital low pass filter to 5Hz
    // Options: MPU9250_DLPF_0, MPU9250_DLPF_1, MPU9250_DLPF_2, 
    //          MPU9250_DLPF_3, MPU9250_DLPF_4, MPU9250_DLPF_5, MPU9250_DLPF_6
    IMU.setGyrDLPF(MPU9250_DLPF_5);
    
    // Set sample rate divider (0 = 1kHz, 1 = 500Hz, etc.)
    IMU.setSampleRateDivider(0);
    
    // Initialize magnetometer if available
    #ifdef MPU9250_WE_USE_MAG
    IMU.initMagnetometer();
    #endif
}

void readIMU() {
	xyzFloat accel = IMU.getGyrValues();
	gyro.x = accel.x * DEG_TO_RAD;
	gyro.y = accel.y * DEG_TO_RAD;
	gyro.z = accel.z * DEG_TO_RAD;
	
	accel = IMU.getAccRawValues();
	acc.x = accel.x * 9.80665; // convert g to m/s²
	acc.y = accel.y * 9.80665;
	acc.z = accel.z * 9.80665;
	calibrateGyroOnce();
	// apply scale and bias
	acc = (acc - accBias) / accScale;
	gyro = gyro - gyroBias;
	// rotate
	rotateIMU(acc);
	rotateIMU(gyro);
}

void rotateIMU(Vector& data) {
	// Rotate from LFD to FLU
	// NOTE: In case of using other IMU orientation, change this line:
	data = Vector(data.y, data.x, -data.z);
	// Axes orientation for various boards: https://github.com/okalachev/flixperiph#imu-axes-orientation
}

void calibrateGyroOnce() {
	static float landedTime = 0;
	landedTime = landed ? landedTime + dt : 0;
	if (landedTime < 2) return; // calibrate only if definitely stationary

	static LowPassFilter<Vector> gyroCalibrationFilter(0.001);
	gyroBias = gyroCalibrationFilter.update(gyro);
}

void calibrateAccel() {
	print("Calibrating accelerometer\n");
	IMU.setAccRange(MPU9250_ACC_RANGE_2G); // the most sensitive mode

	print("1/6 Place level [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("2/6 Place nose up [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("3/6 Place nose down [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("4/6 Place on right side [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("5/6 Place on left side [8 sec]\n");
	pause(8);
	calibrateAccelOnce();
	print("6/6 Place upside down [8 sec]\n");
	pause(8);
	calibrateAccelOnce();

	printIMUCal();
	print("✓ Calibration done!\n");
	configureIMU();
}

void calibrateAccelOnce() {
	const int samples = 1000;
	static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
	static Vector accMin(INFINITY, INFINITY, INFINITY);

	// Compute the average of the accelerometer readings
	acc = Vector(0, 0, 0);
	for (int i = 0; i < samples; i++) {
		xyzFloat sample_acc = IMU.getAccRawValues();
		Vector sample(
			sample_acc.x * 9.80665,  // convert g to m/s²
			sample_acc.y * 9.80665,
			sample_acc.z * 9.80665
		);
		acc = acc + sample;
	}
	acc = acc / samples;

	// Update the maximum and minimum values
	if (acc.x > accMax.x) accMax.x = acc.x;
	if (acc.y > accMax.y) accMax.y = acc.y;
	if (acc.z > accMax.z) accMax.z = acc.z;
	if (acc.x < accMin.x) accMin.x = acc.x;
	if (acc.y < accMin.y) accMin.y = acc.y;
	if (acc.z < accMin.z) accMin.z = acc.z;
	// Compute scale and bias
	accScale = (accMax - accMin) / 2 / ONE_G;
	accBias = (accMax + accMin) / 2;
}

void printIMUCal() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
	// Check if we can read data from the IMU
	bool dataReady = IMU.readAndClearInterrupts() & 0x01;
	print("status: %s\n", dataReady ? "READY" : "NOT READY");
	print("who am I: 0x%02X\n", IMU.whoAmI());
}
