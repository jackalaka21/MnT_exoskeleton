#pragma once

#include <Arduino.h>
#include <Wire.h>

// E-stop Pin
const unsigned int PIN_ESTOP = 2;

// IMU Pins
// inline: C++17 — allows safe inclusion in multiple .cpp files (one definition rule)
// constexpr: addresses are compile-time constants, safe in headers
inline TwoWire* const I2C_Bus      = &Wire;
constexpr uint8_t     IMU1_address = 0x68;
constexpr uint8_t     IMU2_address = 0x69;

//FSR Pins
const unsigned int LEFT_HEEL_FSR_PIN  = A3;
const unsigned int LEFT_TOE_FSR_PIN   = A2;
const unsigned int RIGHT_HEEL_FSR_PIN = A1;
const unsigned int RIGHT_TOE_FSR_PIN  = A0;

// Motor Driver — CAN bus
// CAN2 on Teensy 4.1 is fixed to TX=D1/pin1 / RX=D0/pin0 (via SN65HVD230 transceiver); no pin config needed.
// Node addresses must match `CANCommander commander(can, N)` in each drive's firmware.
constexpr uint8_t  MOTOR_NODE_L = 15;        // bench drive currently flashed as node 15
constexpr uint8_t  MOTOR_NODE_R = 16;        // TODO: set once the second drive is flashed
constexpr uint32_t MOTOR_CAN_BAUD = 1000000; // 1 Mbps — must match CANCommander baudrate