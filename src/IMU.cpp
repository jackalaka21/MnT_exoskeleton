#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

#include "IMU.h"

// Helper Functions
// --------------------------------------------------------------------------------------------
// IMU setup and initialization (return False when failed)
bool IMU::_begin() {
    return _mpu.begin(_address, _wireBus);
}

// 6050 IMU Configuration
void IMU::_configure() {
    _mpu.setAccelerometerRange(MPU6050_RANGE_2_G); // Accelerometer Range
    _mpu.setGyroRange(MPU6050_RANGE_250_DEG);      // Gyroscope Range
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // Filter Bandwidth
}

// Read data from the IMU sensor and update internal struct.
// Direct Wire burst read starting at ACCEL_XOUT_H (0x3B) — 14 bytes:
//   [0..5]  accel X,Y,Z  (2 bytes each, big-endian signed 16-bit)
//   [6..7]  temperature   (skipped)
//   [8..13] gyro  X,Y,Z  (2 bytes each, big-endian signed 16-bit)
// Scale factors for default config (±2g, ±250 °/s): 16384 LSB/g, 131 LSB/°/s.
void IMU::_readData() {
    // STOP (true) then re-START is used instead of repeated-start: the Teensy 4.1
    // LPI2C peripheral returns error 4 on repeated-start under FreeRTOS scheduling.
    // MPU6500 holds the register pointer across a STOP so burst reads still work.
    _wireBus->beginTransmission(_address);
    _wireBus->write(0x3B);

    if (_wireBus->endTransmission(true) != 0) return;

    if (_wireBus->requestFrom(_address, (uint8_t)14) != 14) return;

    int16_t rawAx = ((int16_t)_wireBus->read() << 8) | _wireBus->read();
    int16_t rawAy = ((int16_t)_wireBus->read() << 8) | _wireBus->read();
    int16_t rawAz = ((int16_t)_wireBus->read() << 8) | _wireBus->read();

    _wireBus->read(); _wireBus->read();  // Discard temperature data

    int16_t rawGx = ((int16_t)_wireBus->read() << 8) | _wireBus->read();
    int16_t rawGy = ((int16_t)_wireBus->read() << 8) | _wireBus->read();
    int16_t rawGz = ((int16_t)_wireBus->read() << 8) | _wireBus->read();

    _accData.x  = (rawAx / IMU_ACCEL_SCALE) * IMU_GRAVITY;
    _accData.y  = (rawAy / IMU_ACCEL_SCALE) * IMU_GRAVITY;
    _accData.z  = (rawAz / IMU_ACCEL_SCALE) * IMU_GRAVITY;
    _gyroData.x = rawGx / IMU_GYRO_SCALE;
    _gyroData.y = rawGy / IMU_GYRO_SCALE;
    _gyroData.z = rawGz / IMU_GYRO_SCALE;
}

// Constructor
// --------------------------------------------------------------------------------------------
IMU::IMU(String name, uint8_t address, TwoWire* bus) {
    _sensorName = name;
    _address    = address;
    _wireBus    = bus;
}

// Public Methods
// --------------------------------------------------------------------------------------------
// Initialize the IMU sensor and configure it
void IMU::init() {
    if (_begin() == false) {
        Serial.print("Failed to initialize IMU: ");
        Serial.println(_sensorName);
        _initialized = false;
    }
    else {
        Serial.print("Successfully initialized IMU: ");
        Serial.println(_sensorName);
        _configure();
        _initialized = true;
    }
}

// Read latest sensor data and write directly into the VolatileData.cpp
void IMU::read(volatile AccelData* accel, volatile GyroData* gyro) {
    if (!_initialized) return;
    _readData();
    accel->x = _accData.x;
    accel->y = _accData.y;
    accel->z = _accData.z;
    gyro->x  = _gyroData.x;
    gyro->y  = _gyroData.y;
    gyro->z  = _gyroData.z;
}


void IMU::printData() {
    Serial.print("["); Serial.print(_sensorName); Serial.print("]");
    Serial.print("  Accel(m/s^2): ");
    Serial.print(_accData.x, 2); Serial.print(", ");
    Serial.print(_accData.y, 2); Serial.print(", ");
    Serial.print(_accData.z, 2);
    Serial.print("  Gyro(rad/s): ");
    Serial.print(_gyroData.x, 2); Serial.print(", ");
    Serial.print(_gyroData.y, 2); Serial.print(", ");
    Serial.println(_gyroData.z, 2);
}

void IMU::teleplot() {
    // Spaces in variable names break Teleplot's parser — replace with underscores
    String prefix = _sensorName;
    prefix.replace(' ', '_');

    // Teleplot format: >namespace/variable:value\n
    Serial.print(">"); Serial.print(prefix); Serial.print("/accel_x:"); Serial.println(_accData.x, 4);
    Serial.print(">"); Serial.print(prefix); Serial.print("/accel_y:"); Serial.println(_accData.y, 4);
    Serial.print(">"); Serial.print(prefix); Serial.print("/accel_z:"); Serial.println(_accData.z, 4);
    Serial.print(">"); Serial.print(prefix); Serial.print("/gyro_x:");  Serial.println(_gyroData.x, 4);
    Serial.print(">"); Serial.print(prefix); Serial.print("/gyro_y:");  Serial.println(_gyroData.y, 4);
    Serial.print(">"); Serial.print(prefix); Serial.print("/gyro_z:");  Serial.println(_gyroData.z, 4);
}

