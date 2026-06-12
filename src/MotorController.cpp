#include "MotorController.h"

// CAN1 on Teensy 4.1: TX = pin 22, RX = pin 23.
// Wire each pin through a SN65HVD230 (3.3 V) transceiver to the XDrive Mini CAN connector.
// 120 Ω termination at each physical end of the bus.
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> _can;

MotorController::MotorController(uint8_t node_id_left, uint8_t node_id_right)
    : _id_left(node_id_left), _id_right(node_id_right) {}

void MotorController::init(uint32_t baud_rate) {
    _can.begin();
    _can.setBaudRate(baud_rate);
}

void MotorController::enable() {
    _writeByte(_id_left,  SFOCCan::REG_ENABLE, 0x01);
    _writeByte(_id_right, SFOCCan::REG_ENABLE, 0x01);
}

void MotorController::disable() {
    _writeByte(_id_left,  SFOCCan::REG_ENABLE, 0x00);
    _writeByte(_id_right, SFOCCan::REG_ENABLE, 0x00);
}

void MotorController::setTorque(float tau_left, float tau_right) {
    // Clamp symmetrically — firmware (canWriteTorque) only clamps the positive side.
    constexpr float LIM = SFOCCan::TORQUE_MAX_NM;
    tau_left  = constrain(tau_left,  -LIM, LIM);
    tau_right = constrain(tau_right, -LIM, LIM);
    _writeFloat(_id_left,  SFOCCan::REG_TORQUE, tau_left);
    _writeFloat(_id_right, SFOCCan::REG_TORQUE, tau_right);
}

void MotorController::requestStates() {
    _readRequest(_id_left,  SFOCCan::REG_ANGLE);
    _readRequest(_id_left,  SFOCCan::REG_VELOCITY);
    _readRequest(_id_right, SFOCCan::REG_ANGLE);
    _readRequest(_id_right, SFOCCan::REG_VELOCITY);
}

bool MotorController::pollStates(MotorState& out_left, MotorState& out_right) {
    CAN_message_t msg;
    bool updated = false;

    while (_can.read(msg)) {
        if (!msg.flags.extended) continue;

        uint8_t node  = (msg.id >> 20) & 0xFF;
        uint8_t ptype = (msg.id >> 16) & 0x0F;
        uint8_t reg   = (msg.id >>  8) & 0xFF;

        if (ptype != SFOCCan::PKT_RESPONSE || msg.len < 4) continue;

        float val;
        memcpy(&val, msg.buf, 4);

        MotorState* dst = (node == _id_left)  ? &out_left  :
                          (node == _id_right) ? &out_right : nullptr;
        if (!dst) continue;

        if (reg == SFOCCan::REG_ANGLE)    { dst->angle    = val; updated = true; }
        if (reg == SFOCCan::REG_VELOCITY) { dst->velocity = val; updated = true; }
    }
    return updated;
}

bool MotorController::readRaw(uint32_t& out_id, bool& out_extended, uint8_t& out_len, uint8_t out_buf[8]) {
    CAN_message_t msg;
    if (!_can.read(msg)) return false;
    out_id       = msg.id;
    out_extended = msg.flags.extended;
    out_len      = msg.len;
    memcpy(out_buf, msg.buf, msg.len);
    return true;
}

// ─── Private helpers ────────────────────────────────────────────────────────

uint32_t MotorController::_canId(uint8_t node, uint8_t pkt_type, uint8_t reg, uint8_t motor_idx) {
    return ((uint32_t)node << 20) | ((uint32_t)pkt_type << 16) | ((uint32_t)reg << 8) | motor_idx;
}

void MotorController::_writeFloat(uint8_t node, uint8_t reg, float value) {
    CAN_message_t msg = {};
    msg.flags.extended = 1;
    msg.id  = _canId(node, SFOCCan::PKT_WRITE, reg);
    msg.len = 4;
    memcpy(msg.buf, &value, 4);
    _can.write(msg);
}

void MotorController::_writeByte(uint8_t node, uint8_t reg, uint8_t value) {
    CAN_message_t msg = {};
    msg.flags.extended = 1;
    msg.id  = _canId(node, SFOCCan::PKT_WRITE, reg);
    msg.len = 1;
    msg.buf[0] = value;
    _can.write(msg);
}

void MotorController::_readRequest(uint8_t node, uint8_t reg) {
    CAN_message_t msg = {};
    msg.flags.extended = 1;
    msg.id  = _canId(node, SFOCCan::PKT_READ, reg);
    msg.len = 0;
    _can.write(msg);
}
