// #include <Arduino.h>

// // put function declarations here:
// int myFunction(int, int);

// void setup() {
//   // put your setup code here, to run once:
//   int result = myFunction(2, 3);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
// }

// // put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }


// /*
// This code example shows normal operation of the SimpleCANio library.

// This code is specific to Teensy boards using FlexCAN_T4.
// */
// #include <Arduino.h>
// #include "SimpleCANio.h"   // <- this is the only include required, it should be smart enough to find the correct subclass

// #define CAN_ID 0x321

// // | Board        | CAN bus|  CAN RX Pin | CAN TX Pin |
// // |--------------|-------------|------------|------------|
// // | Teensy 3.2   |CAN0         | 4          | 3          |
// // | Teensy 3.5   |CAN0         | 4          | 3          |
// // | Teensy 3.6   |CAN0         | 4          | 3          |
// // | Teensy 3.6   |CAN1         | 34         | 33         |
// // | Teensy 4.0   |CAN1         | 23         | 22         |
// // | Teensy 4.1   |CAN2         | 0          | 1          |
// // | Teensy 4.1   |CAN3         | 30         | 31         |    
// #define CAN_BUS CAN1 // TODO set the bus you want to use , Teensy
// #define CAN_SHDN NC
// #define CAN_ENABLE NC

// auto can_bus = TEENSY_FLEXCAN(CAN_BUS);
// CANio can(can_bus, CAN_SHDN, CAN_ENABLE); 

// void setup()
// {

//     Serial.begin(230400);

//     can.logTo(&Serial);
//     delay(2000);
//     Serial.println("Starting CAN");
//     // can.enableInternalLoopback();
    
//     // choose one of the receive filters to apply
//     CanFilter filter = CanFilter(MASK_EXTENDED, CAN_ID, CAN_ID, FILTER_ANY_FRAME);
//     // CanFilter filter = CanFilter(MASK_STANDARD, CAN_ID, CAN_ID, FILTER_ANY_FRAME);
//     // CanFilter filter = CanFilter(MASK_ACCEPT_ALL);
//     can.filter(filter);

//     // 1 mbps
//     can.begin(1000000);
//     delay(10);
// }

// uint8_t data[8] = {0};
// uint8_t num = 0;

// void loop()
// {

//     data[0] = num++;

//     bool isExtendedFrame = false;
//     bool isRtr = false;
//     delay(50);
//     CanMsg txMsg = CanMsg(
//         isExtendedFrame ? CanExtendedId(CAN_ID, isRtr) : CanStandardId(CAN_ID, isRtr),
//         4,
//         data);
//     delay(50);

//     can.write(txMsg);
//     Serial.print("TX: ");
//     Serial.println(txMsg);
//     delay(1);

//     if (can.available() > 0)
//     {
//         CanMsg const rxMsg = can.read();

//         Serial.print("polling read: ");
//         if (rxMsg.isExtendedId())
//         {
//             Serial.print(rxMsg.getExtendedId(), HEX);
//             Serial.println(" Extended ✅");
//         }
//         else
//         {
//             Serial.print(rxMsg.getStandardId(), HEX);
//             Serial.println(" Standard ✅");
//         }
//     }

//     delay(2000);
// }


#include <Arduino.h>
#include "SimpleCANio.h"

// Set to match your Teensy board:
// Teensy 4.0 → CAN1 (TX=22, RX=23)
// Teensy 4.1 → CAN2 (TX=1,  RX=0) or CAN3 (TX=31, RX=30)
// Teensy 3.x → CAN0 (TX=3,  RX=4)
#define CAN_BUS CAN1

auto flexcan = TEENSY_FLEXCAN(CAN_BUS);
CANio can(flexcan);

// Protocol constants — must match CANCommander.h
#define CAN_ADDRESS_SHIFT     20
#define CAN_PACKET_TYPE_SHIFT 16
#define CAN_REGISTER_SHIFT     8
#define CAN_MOTOR_SHIFT        0

#define CAN_READ_REQUEST  0x1
#define CAN_WRITE_REQUEST 0x2
#define CAN_READ_RESPONSE 0x3

// Registers — from SimpleFOCRegisters.h
#define REG_STATUS   0x00  // RO  4 bytes
#define REG_TARGET   0x01  // RW  float
#define REG_ENABLE   0x04  // RW  uint8
#define REG_MOTION_TYPE 0x07  // RW  uint8 (0=torque,1=velocity,2=angle)
#define REG_ANGLE    0x09  // RO  float
#define REG_VELOCITY 0x11  // RO  float

// Must match CANCommander constructor: CANCommander commandc(can, 15)
#define MOTOR_NODE_ADDR 15

// ---- helpers ----

uint32_t makeId(uint8_t addr, uint8_t ptype, uint8_t reg, uint8_t motor = 0) {
    return ((uint32_t)addr  << CAN_ADDRESS_SHIFT)     |
           ((uint32_t)ptype << CAN_PACKET_TYPE_SHIFT) |
           ((uint32_t)reg   << CAN_REGISTER_SHIFT)    |
           ((uint32_t)motor << CAN_MOTOR_SHIFT);
}

void canRecover() {
    Serial.println("CAN bus-off — reinitialising...");
    can.end();
    delay(100);
    can.begin(1000000);
    delay(100);
}

void writeFloat(uint8_t reg, float value, uint8_t motor = 0) {
    uint8_t buf[4];
    memcpy(buf, &value, 4);
    uint32_t id = makeId(MOTOR_NODE_ADDR, CAN_WRITE_REQUEST, reg, motor);
    CanMsg msg(CanExtendedId(id), 4, buf);
    int result = can.write(msg);
    if (result != CAN_OK) {
        Serial.println("TX FAILED");
        canRecover();
    }
}

void writeByte(uint8_t reg, uint8_t value, uint8_t motor = 0) {
    uint32_t id = makeId(MOTOR_NODE_ADDR, CAN_WRITE_REQUEST, reg, motor);
    Serial.print("TX WRITE_BYTE id=0x"); Serial.print(id, HEX);
    Serial.print(" reg=0x"); Serial.print(reg, HEX);
    Serial.print(" val="); Serial.println(value);
    can.write(CanMsg(CanExtendedId(id), 1, &value));
}

void requestRead(uint8_t reg, uint8_t motor = 0) {
    uint8_t empty = 0;
    uint32_t id = makeId(MOTOR_NODE_ADDR, CAN_READ_REQUEST, reg, motor);
    Serial.print("TX READ_REQ  id=0x"); Serial.print(id, HEX);
    Serial.print(" reg=0x"); Serial.println(reg, HEX);
    can.write(CanMsg(CanExtendedId(id), 0, &empty));
}

void handleResponse(CanMsg& msg) {
    uint32_t id  = msg.getExtendedId();
    uint8_t ptype = (id >> CAN_PACKET_TYPE_SHIFT) & 0xF;
    uint8_t reg   = (id >> CAN_REGISTER_SHIFT)    & 0xFF;

    if (ptype != CAN_READ_RESPONSE) return;

    switch (reg) {
        case REG_VELOCITY: {
            float v; memcpy(&v, msg.data, 4);
            Serial.print("Velocity: "); Serial.println(v);
            break;
        }
        case REG_ANGLE: {
            float a; memcpy(&a, msg.data, 4);
            Serial.print("Angle: "); Serial.println(a);
            break;
        }
        case REG_STATUS:
            Serial.print("Status — motor:");   Serial.print(msg.data[0]);
            Serial.print("  last_reg:0x");     Serial.print(msg.data[1], HEX);
            Serial.print("  err:");            Serial.print(msg.data[2]);
            Serial.print("  mot_status:");     Serial.println(msg.data[3]);
            break;
    }
}

// ---- Arduino ----

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    can.logTo(&Serial);
    can.filter(CanFilter(MASK_ACCEPT_ALL));
    can.begin(1000000);  // 1 Mbps — must match CANCommander baudrate
    delay(500);

    writeByte(REG_ENABLE, 1);
    delay(50);
    writeByte(REG_MOTION_TYPE, 0);  // torque control mode
    delay(50);
    writeFloat(REG_TARGET, 0.0f);
}

void loop() {
    static uint32_t last = 0;

    if (millis() - last > 5000) {
        writeFloat(REG_TARGET, 0.05f);  // 0.05 A torque command
        last = millis();
    }

    while (can.available() > 0) {
        CanMsg msg = can.read();
        if (msg.isExtendedId())
            handleResponse(msg);
    }
}
