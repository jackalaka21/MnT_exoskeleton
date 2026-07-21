#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

#include "Config.h"
#include "VolatileData.h"

// SimpleFOC CANCommander protocol
// --------------------------------------------------------------------------------------------
// 29-bit extended CAN ID layout — must match CANCommander.h on the drive:
//   [27:20] node address | [19:16] packet type | [15:8] register | [7:0] motor index
namespace SFOCCan {
    // ID field shifts
    constexpr uint8_t ADDRESS_SHIFT     = 20;
    constexpr uint8_t PACKET_TYPE_SHIFT = 16;
    constexpr uint8_t REGISTER_SHIFT    = 8;
    constexpr uint8_t MOTOR_SHIFT       = 0;

    // Packet types
    constexpr uint8_t READ_REQUEST  = 0x1;
    constexpr uint8_t WRITE_REQUEST = 0x2;
    constexpr uint8_t READ_RESPONSE = 0x3;

    // Registers — from SimpleFOCRegisters.h
    constexpr uint8_t REG_STATUS      = 0x00;  // RO  4 bytes: motor, last_reg, err, mot_status
    constexpr uint8_t REG_TARGET      = 0x01;  // RW  float — target in the active motion mode
    constexpr uint8_t REG_ENABLE      = 0x04;  // RW  uint8: 0=off, 1=on
    constexpr uint8_t REG_MOTION_TYPE = 0x05;  // RW  uint8: see MOTION_* below (REG_CONTROL_MODE in SimpleFOCRegisters.h)
    constexpr uint8_t REG_ANGLE       = 0x09;  // RO  float, rad
    constexpr uint8_t REG_VELOCITY    = 0x11;  // RO  float, rad/s

    // REG_MOTION_TYPE values
    constexpr uint8_t MOTION_TORQUE   = 0;
    constexpr uint8_t MOTION_VELOCITY = 1;
    constexpr uint8_t MOTION_ANGLE    = 2;

    // Assemble a 29-bit extended CAN ID from its fields
    constexpr uint32_t makeId(uint8_t addr, uint8_t ptype, uint8_t reg, uint8_t motor = 0) {
        return ((uint32_t)addr  << ADDRESS_SHIFT)     |
               ((uint32_t)ptype << PACKET_TYPE_SHIFT) |
               ((uint32_t)reg   << REGISTER_SHIFT)    |
               ((uint32_t)motor << MOTOR_SHIFT);
    }
}

// MKS XDrive Mini + gearbox parameters — for joint-side ↔ motor-side conversion
// --------------------------------------------------------------------------------------------
// Encoder is on the motor shaft (before the gearbox), so the drive reports motor-side
// angle/velocity and takes a motor current target. Conversions:
//   joint angle  = motor angle  / GEAR_RATIO
//   joint torque = current × NM_PER_AMP   (measured, ≈ 3.9 Nm per amp)
// Values are defined in Config.h → Config::Motor (edit them there). These aliases keep the
// XDrive::NAME spelling used throughout MotorCAN.cpp while Config stays the single source of truth.
namespace XDrive {
    constexpr float KT         = Config::Motor::KT;
    constexpr float GEAR_RATIO = Config::Motor::GEAR_RATIO;
    constexpr float GEAR_EFF   = Config::Motor::GEAR_EFF;
    constexpr float I_MAX      = Config::Motor::I_MAX;
    constexpr float NM_PER_AMP = Config::Motor::NM_PER_AMP;
}

// MotorCAN — Teensy-side master for SimpleFOC drives on CAN2 (TX=D1/pin1, RX=D0/pin0)
// --------------------------------------------------------------------------------------------
class MotorCAN {
    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String   _busName;                              // For clean debug logs
        uint32_t _baud = 1000000;                       // Saved for bus-off recovery
        FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> _can;

        // Latest feedback per node, filled by poll()
        struct Feedback {
            float    angle      = 0.0f;   // rad
            float    velocity   = 0.0f;   // rad/s
            uint8_t  status[4]  = {0};    // raw REG_STATUS bytes
            uint32_t last_rx_ms = 0;      // millis() of last READ_RESPONSE
        };
        static constexpr uint8_t MAX_NODES = 2;
        uint8_t  _nodes[MAX_NODES];
        Feedback _fb[MAX_NODES];

        uint32_t _last_recover_ms = 0;
        bool     _armed = false;         // true once startup() has both drives online + enabled

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // Map a node address to its feedback slot; -1 if not one of ours
        int _slotFor(uint8_t node) const;

        // Send one extended frame; true if it was queued into a TX mailbox
        bool _write(uint32_t id, const uint8_t* data, uint8_t len);

        // Re-init the controller if the hardware reports Bus Off (rate-limited)
        void _recoverIfBusOff();

    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        MotorCAN(String name, uint8_t node_left, uint8_t node_right);

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Bus setup: begin → baud → FIFO → accept-all filter.
        // Polled RX only — enableFIFOInterrupt() must NOT be used, it makes read() skip the FIFO.
        void init(uint32_t baud = 1000000);

        // High-level bring-up — ALL-OR-NOTHING. Waits for every drive to answer, then puts each in
        // torque mode at a zero target and enables it. If ANY drive fails to respond, all drives are
        // disabled and this returns false, so the two legs can never run half-configured. Call after
        // init(). Blocks up to timeout_ms per drive (firmware needs ~10 s to boot before it serves CAN).
        bool startup(uint32_t timeout_ms = 30000);

        // Zero every target and disable every drive; clears the armed flag. Use on E-stop / fault.
        void disableAll();

        // True once startup() has brought both drives online and enabled.
        bool armed() const { return _armed; }

        // Write commands
        bool enable (uint8_t node, uint8_t motor = 0);
        bool disable(uint8_t node, uint8_t motor = 0);
        bool setMotionType(uint8_t node, uint8_t type, uint8_t motor = 0);
        bool setTarget(uint8_t node, float value, uint8_t motor = 0);   // raw REG_TARGET (amps in torque mode)

        // Command joint torque in Nm — converts to a motor current target (Nm → A through
        // the gearbox) and clamps to ±I_MAX before sending. Use this from the control loop.
        bool setTorqueNm(uint8_t node, float tau_joint_nm, uint8_t motor = 0);

        // Read requests — drive answers asynchronously; poll() picks up the response
        bool requestAngle   (uint8_t node, uint8_t motor = 0);
        bool requestVelocity(uint8_t node, uint8_t motor = 0);
        bool requestStatus  (uint8_t node, uint8_t motor = 0);

        // Block until the node answers a status request (true) or timeout_ms elapses (false).
        // Use before configuring a drive: its firmware spends ~10 s in setup() (motor
        // characterisation) before it services CAN, so early writes would be lost.
        bool waitForNode(uint8_t node, uint32_t timeout_ms, uint8_t motor = 0);

        // Drain the RX FIFO and update per-node feedback. Call every control cycle.
        void poll();

        // Copy latest feedback for one node into the VolatileData globals,
        // converted to joint side (drive reports motor shaft; ÷ GEAR_RATIO)
        void read(uint8_t node, volatile float* angle, volatile float* velocity);

        // millis() timestamp of the last response from a node (0 = never heard from it)
        uint32_t lastResponseMs(uint8_t node) const;

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printStatus(uint8_t node);   // last REG_STATUS payload
        void printBusState();             // FLEXCAN2 ESR1/ECR: fault state + TX/RX error counters
};
