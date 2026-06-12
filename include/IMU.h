#pragma once

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "VolatileData.h"

// Scale factors matching _configure() settings (±2g, ±250 °/s)
constexpr float IMU_ACCEL_SCALE = 16384.0f;   // LSB/g
constexpr float IMU_GRAVITY     = 9.80665f;   // m/s²
constexpr float IMU_GYRO_SCALE  = 7509.87f;   // LSB/(rad/s)  =  131 LSB/°/s × 180/π

class IMU {
    private:
        // Internal Variables
        // --------------------------------------------------------------------------------------------
        // Sensor configuration
        Adafruit_MPU6050 _mpu;        // Object composition instead of inheritance (Adafruit Library)
        uint8_t          _address;    // Sensor I2C address (0x68 or 0x69) (Check hardware description)
        TwoWire*         _wireBus;    // Pointer to the chosen Wire bus
        String           _sensorName; // For clean debug logs
        bool             _initialized = false;

        // Sensor Data
        struct { float x; float y; float z; } _accData;
        struct { float x; float y; float z; } _gyroData;

        // Helper Functions
        // --------------------------------------------------------------------------------------------
        bool _begin();      // IMU setup and initialization (return False when failed)
        void _configure();  // 6050 IMU Configuration
        void _readData();   // Read sensor and update _accData / _gyroData

    public:
        // Constructor
        // --------------------------------------------------------------------------------------------
        IMU(String name, uint8_t address, TwoWire* bus);

        // Public Methods
        // --------------------------------------------------------------------------------------------
        void init(); // Initialize the IMU sensor and configure it
        void read(volatile AccelData* accel, volatile GyroData* gyro);  // Read latest sensor data and write directly into the VolatileData.cpp

        // Serial Monitor / Teleplot (for debugging only)
        // --------------------------------------------------------------------------------------------
        void printData();
        void teleplot(); // Outputs Teleplot-compatible serial lines: >name/var:value|unit
};