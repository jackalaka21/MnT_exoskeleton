#include "MotorCAN.h"

// Constructor
// --------------------------------------------------------------------------------------------
MotorCAN::MotorCAN(String name, uint8_t node_left, uint8_t node_right)
    : _busName(name) {
    _nodes[0] = node_left;
    _nodes[1] = node_right;
}

// Helper Functions
// --------------------------------------------------------------------------------------------
int MotorCAN::_slotFor(uint8_t node) const {
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (_nodes[i] == node) return i;
    }
    return -1;
}

bool MotorCAN::_write(uint32_t id, const uint8_t* data, uint8_t len) {
    CAN_message_t msg;
    msg.id = id;
    msg.flags.extended = 1;   // SimpleFOC CANCommander uses 29-bit IDs only
    msg.len = len;
    if (len > 0) memcpy(msg.buf, data, len);
    return _can.write(msg) == 1;
}

void MotorCAN::_recoverIfBusOff() {
    uint32_t esr1 = *(volatile uint32_t*)(IMXRT_FLEXCAN2_ADDRESS + 0x20);
    uint8_t fltconf = (esr1 >> 4) & 0x3;   // 0=Error Active, 1=Error Passive, 2/3=Bus Off
    if (fltconf < 2) return;

    if (millis() - _last_recover_ms < 500) return;
    _last_recover_ms = millis();

    Serial.print(_busName);
    Serial.println(": bus off — reinitialising");
    init(_baud);
}

// Public Methods
// --------------------------------------------------------------------------------------------
void MotorCAN::init(uint32_t baud) {
    _baud = baud;

    _can.begin();
    _can.setBaudRate(baud);   // must match CANCommander baudrate on the drive
    _can.setMaxMB(16);
    _can.enableFIFO();
    // enableFIFO() with no filter rejects ALL frames — accept-all is mandatory
    _can.setFIFOFilter(ACCEPT_ALL);
    // Do NOT call enableFIFOInterrupt(): with the FIFO interrupt mask set,
    // read() falls through to the (empty) mailboxes and frames pile up unseen.
}

bool MotorCAN::startup(uint32_t timeout_ms) {
    _armed = false;

    // All-or-nothing: every drive must answer before we enable anything, so the two legs can
    // never run half-configured. A single missing/silent drive leaves the whole system disabled.
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        if (!waitForNode(_nodes[i], timeout_ms)) {
            Serial.print(_busName);
            Serial.print(": drive node ");
            Serial.print(_nodes[i]);
            Serial.println(" not found — check wiring/power/node ID. ALL motors stay disabled.");
            disableAll();
            return false;
        }
    }

    // Every drive answered — put each in torque mode at a zero target, then enable it. The firmware
    // needs a moment to apply the mode/target writes before the enable, hence the short delays.
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        setMotionType(_nodes[i], SFOCCan::MOTION_TORQUE);
        delay(50);
        setTarget(_nodes[i], 0.0f);
        delay(50);
        enable(_nodes[i]);
    }

    _armed = true;
    return true;
}

void MotorCAN::disableAll() {
    for (uint8_t i = 0; i < MAX_NODES; i++) {
        setTarget(_nodes[i], 0.0f);
        disable(_nodes[i]);
    }
    _armed = false;
}

bool MotorCAN::enable(uint8_t node, uint8_t motor) {
    uint8_t on = 1;
    return _write(SFOCCan::makeId(node, SFOCCan::WRITE_REQUEST, SFOCCan::REG_ENABLE, motor), &on, 1);
}

bool MotorCAN::disable(uint8_t node, uint8_t motor) {
    uint8_t off = 0;
    return _write(SFOCCan::makeId(node, SFOCCan::WRITE_REQUEST, SFOCCan::REG_ENABLE, motor), &off, 1);
}

bool MotorCAN::setMotionType(uint8_t node, uint8_t type, uint8_t motor) {
    return _write(SFOCCan::makeId(node, SFOCCan::WRITE_REQUEST, SFOCCan::REG_MOTION_TYPE, motor), &type, 1);
}

bool MotorCAN::setTarget(uint8_t node, float value, uint8_t motor) {
    uint8_t buf[4];
    memcpy(buf, &value, 4);
    return _write(SFOCCan::makeId(node, SFOCCan::WRITE_REQUEST, SFOCCan::REG_TARGET, motor), buf, 4);
}

bool MotorCAN::setTorqueNm(uint8_t node, float tau_joint_nm, uint8_t motor) {
    float amps = tau_joint_nm / XDrive::NM_PER_AMP;
    amps = constrain(amps, -XDrive::I_MAX, XDrive::I_MAX);
    return setTarget(node, amps, motor);
}

bool MotorCAN::requestAngle(uint8_t node, uint8_t motor) {
    return _write(SFOCCan::makeId(node, SFOCCan::READ_REQUEST, SFOCCan::REG_ANGLE, motor), nullptr, 0);
}

bool MotorCAN::requestVelocity(uint8_t node, uint8_t motor) {
    return _write(SFOCCan::makeId(node, SFOCCan::READ_REQUEST, SFOCCan::REG_VELOCITY, motor), nullptr, 0);
}

bool MotorCAN::requestStatus(uint8_t node, uint8_t motor) {
    return _write(SFOCCan::makeId(node, SFOCCan::READ_REQUEST, SFOCCan::REG_STATUS, motor), nullptr, 0);
}

bool MotorCAN::waitForNode(uint8_t node, uint32_t timeout_ms, uint8_t motor) {
    int slot = _slotFor(node);
    if (slot < 0) return false;

    uint32_t start = millis();
    uint32_t heard_before = _fb[slot].last_rx_ms;

    while (millis() - start < timeout_ms) {
        requestStatus(node, motor);
        // Give the drive time to answer; re-request every 100 ms in case the
        // frame arrived while its RX FIFO was full or it was still booting
        uint32_t deadline = millis() + 100;
        while (millis() < deadline) {
            poll();
            if (_fb[slot].last_rx_ms != heard_before) {
                Serial.print(_busName);
                Serial.print(": node ");
                Serial.print(node);
                Serial.println(" online");
                return true;
            }
            delay(1);
        }
    }
    Serial.print(_busName);
    Serial.print(": node ");
    Serial.print(node);
    Serial.println(" NOT responding");
    return false;
}

void MotorCAN::poll() {
    _recoverIfBusOff();

    CAN_message_t msg;
    while (_can.read(msg)) {
        if (!msg.flags.extended) continue;

        uint8_t node  = (msg.id >> SFOCCan::ADDRESS_SHIFT)     & 0xFF;
        uint8_t ptype = (msg.id >> SFOCCan::PACKET_TYPE_SHIFT) & 0xF;
        uint8_t reg   = (msg.id >> SFOCCan::REGISTER_SHIFT)    & 0xFF;

        if (ptype != SFOCCan::READ_RESPONSE) continue;

        int slot = _slotFor(node);
        if (slot < 0) continue;

        switch (reg) {
            case SFOCCan::REG_ANGLE:
                if (msg.len >= 4) memcpy(&_fb[slot].angle, msg.buf, 4);
                break;
            case SFOCCan::REG_VELOCITY:
                if (msg.len >= 4) memcpy(&_fb[slot].velocity, msg.buf, 4);
                break;
            case SFOCCan::REG_STATUS:
                if (msg.len >= 4) memcpy(_fb[slot].status, msg.buf, 4);
                break;
            default:
                continue;   // unhandled register — don't refresh the timestamp
        }
        _fb[slot].last_rx_ms = millis();
    }
}

void MotorCAN::read(uint8_t node, volatile float* angle, volatile float* velocity) {
    int slot = _slotFor(node);
    if (slot < 0) return;
    // Encoder is on the motor shaft, before the gearbox — scale down to joint side
    *angle    = _fb[slot].angle    / XDrive::GEAR_RATIO;
    *velocity = _fb[slot].velocity / XDrive::GEAR_RATIO;
}

uint32_t MotorCAN::lastResponseMs(uint8_t node) const {
    int slot = _slotFor(node);
    return (slot < 0) ? 0 : _fb[slot].last_rx_ms;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void MotorCAN::printStatus(uint8_t node) {
    int slot = _slotFor(node);
    if (slot < 0) return;
    Serial.print(_busName);
    Serial.print(" node ");        Serial.print(node);
    Serial.print(" — motor:");     Serial.print(_fb[slot].status[0]);
    Serial.print("  last_reg:0x"); Serial.print(_fb[slot].status[1], HEX);
    Serial.print("  err:");        Serial.print(_fb[slot].status[2]);
    Serial.print("  mot_status:"); Serial.print(_fb[slot].status[3]);
    Serial.print("  last_rx_ms:"); Serial.println(_fb[slot].last_rx_ms);
}

void MotorCAN::printBusState() {
    // Hardware registers are authoritative — _can.error() only reports ISR-captured transitions
    uint32_t esr1 = *(volatile uint32_t*)(IMXRT_FLEXCAN2_ADDRESS + 0x20);
    uint32_t ecr  = *(volatile uint32_t*)(IMXRT_FLEXCAN2_ADDRESS + 0x1C);
    uint8_t fltconf = (esr1 >> 4) & 0x3;

    Serial.print(_busName);
    Serial.print(": ");
    switch (fltconf) {
        case 0:  Serial.print("Error Active");  break;
        case 1:  Serial.print("Error Passive"); break;
        default: Serial.print("BUS OFF");       break;
    }
    // TEC pinned high / Bus Off → our TX not ACKed (wiring/baud/no live node).
    // TEC=0 with no response → drive side (wrong node ID, drive not running).
    Serial.print("  TEC:"); Serial.print(ecr & 0xFF);
    Serial.print("  REC:"); Serial.println((ecr >> 8) & 0xFF);
}
