// CAN bus diagnostic — standalone, no FreeRTOS / no motor control.
//
// Continuously pings drive nodes 15 & 16 and reports, once per cycle:
//   • whether each node answers a status request (drive alive + bus OK end-to-end), and
//   • the FLEXCAN2 bus state: fault level + TX/RX error counters (TEC/REC).
//
// Flash with the dedicated env (does not touch the main firmware):
//     pio run -e teensy41_can -t upload
//     pio device monitor -b 115200
//
// Reading the output:
//   nodeNN OK                         → that drive is answering; the CAN link to it is healthy.
//   nodeNN --                         → no response from that drive.
//   repeated "bus off — reinitialising"
//     or TEC climbing / "BUS OFF"     → our frames aren't being ACKed: wiring/termination, wrong
//                                       baud, or NO live drive on the bus at all.
//   "Error Active", TEC 0, but nodes --→ bus wiring is fine, the drive just isn't talking
//                                       (unpowered, not booted, or wrong node ID).

#include <Arduino.h>

#include "Pin.h"
#include "MotorCAN.h"

static MotorCAN motor_can("CAN diag", MOTOR_NODE_L, MOTOR_NODE_R);

// A node counts as alive if it answered a status request within the last half-second.
static void reportNode(uint8_t node) {
    uint32_t last  = motor_can.lastResponseMs(node);
    bool     alive = last != 0 && (millis() - last) < 500;
    Serial.print("  node");
    Serial.print(node);
    Serial.print(": ");
    Serial.print(alive ? "OK" : "--");
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) { /* wait for USB serial, but don't hang headless */ }

    Serial.println("CAN bus diagnostic — pinging nodes 15 & 16 at 1 Mbit/s");
    motor_can.init(MOTOR_CAN_BAUD);
}

void loop() {
    // Ping both drives with a status request.
    motor_can.requestStatus(MOTOR_NODE_L);
    motor_can.requestStatus(MOTOR_NODE_R);

    // Drain responses for ~200 ms (poll() also auto-recovers the bus if it goes Bus Off,
    // printing "bus off — reinitialising" — itself a sign our TX isn't being ACKed).
    uint32_t deadline = millis() + 200;
    while (millis() < deadline) {
        motor_can.poll();
        delay(2);
    }

    // Per-node liveness, then the raw FLEXCAN2 bus state.
    Serial.print("Nodes:");
    reportNode(MOTOR_NODE_L);
    reportNode(MOTOR_NODE_R);
    Serial.println();
    motor_can.printBusState();
    Serial.println("----------------------------------------");

    delay(300);
}
