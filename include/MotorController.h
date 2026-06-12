#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

// SimpleFOC CANCommander register protocol (MKS XDrive Mini firmware)
// Extended 29-bit CAN ID: [27:20]=node_addr [19:16]=pkt_type [15:8]=reg [7:0]=motor_idx
namespace SFOCCan {
    constexpr uint8_t PKT_READ     = 0x1;
    constexpr uint8_t PKT_WRITE    = 0x2;
    constexpr uint8_t PKT_RESPONSE = 0x3;

    constexpr uint8_t REG_TARGET   = 0x01;  // R/W float — Iq setpoint [A] in foc_current mode
    constexpr uint8_t REG_ENABLE   = 0x04;  // R/W 1 byte — 0=disable 1=enable
    constexpr uint8_t REG_ANGLE    = 0x09;  // RO float — shaft angle (rad)
    constexpr uint8_t REG_VELOCITY = 0x11;  // RO float — shaft velocity (rad/s)
    constexpr uint8_t REG_TORQUE   = 0xE0;  // R/W float — output torque [Nm], custom register
                                             //   firmware clamps to [0, TORQUE_OUT_MAX] then
                                             //   converts Nm → Iq [A] = Nm / (GEAR_RATIO * GEAR_EFF * KT)
                                             //   setpoint is rate-limited at 3.0 Nm/s in the FOC ISR

    // Mirror of firmware constants (mks_xdrive_mini_firmware/src/main.cpp).
    // Update here whenever the XDrive Mini firmware values change.
    constexpr float KT          = 0.23f;   // motor torque constant [Nm/A]
    constexpr float GEAR_RATIO  = 15.0f;   // gearbox reduction ratio
    constexpr float GEAR_EFF    = 0.6f;    // gearbox efficiency
    constexpr float I_MAX       = 7.0f;    // peak phase current [A]
    // Maximum output torque the firmware will deliver [Nm] = KT * I_MAX * GEAR_RATIO * GEAR_EFF
    constexpr float TORQUE_MAX_NM = KT * I_MAX * GEAR_RATIO * GEAR_EFF; // ≈ 14.49 Nm
}

struct MotorState {
    float angle;    // rad, at motor shaft
    float velocity; // rad/s, at motor shaft
};

// Controls two MKS XDrive Mini axes over CAN1 (Teensy 4.1: TX=22, RX=23).
// Each XDrive Mini runs SimpleFOC in foc_current (closed-loop current) mode.
// Node IDs are configured per-board in each XDrive Mini's main.cpp (CAN_NODE_ID).
class MotorController {
public:
    // node_id_left / node_id_right: configured in XDrive Mini firmware (CAN_NODE_ID)
    MotorController(uint8_t node_id_left, uint8_t node_id_right);

    // Start CAN1. baud_rate must match XDrive Mini config (default 1 Mbit/s).
    void init(uint32_t baud_rate = 1000000);

    // Enable both axes (REG_ENABLE = 1).
    void enable();

    // Disable both axes (REG_ENABLE = 0). Safe to call any time.
    void disable();

    // Set output torque on both axes via custom register 0xE0.
    // Values are clamped to ±SFOCCan::TORQUE_MAX_NM before transmission.
    // Firmware further rate-limits changes at 3.0 Nm/s and converts Nm → Iq [A].
    void setTorque(float tau_left, float tau_right);

    // Send read requests for angle + velocity on both nodes.
    // Call pollStates() after to collect replies.
    void requestStates();

    // Drain RX FIFO and update out_left / out_right with the latest response frames.
    // Returns true if at least one register was updated.
    bool pollStates(MotorState& out_left, MotorState& out_right);

    // Read one raw frame from the RX FIFO without any filtering.
    // Returns true if a frame was available; fills id, len, buf[8].
    bool readRaw(uint32_t& out_id, bool& out_extended, uint8_t& out_len, uint8_t out_buf[8]);

private:
    uint8_t _id_left;
    uint8_t _id_right;

    uint32_t _canId(uint8_t node, uint8_t pkt_type, uint8_t reg, uint8_t motor_idx = 0);
    void _writeFloat(uint8_t node, uint8_t reg, float value);
    void _writeByte(uint8_t node, uint8_t reg, uint8_t value);
    void _readRequest(uint8_t node, uint8_t reg);
};
