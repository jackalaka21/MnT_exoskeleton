#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include "Pin.h"
#include "VolatileData.h"

#include "IMU.h"
#include "FSR.h"
#include "MadgwickFilter.h"
#include "GaitFSM.h"
#include "Controller.h"
#include "MotorController.h"

// Forward declarations 
// -------------------------------------------------------------------------------------------
void ESTOP_ISR();

// Global Peripherals Objects
// -------------------------------------------------------------------------------------------
// Constructors must NOT make hardware calls (Wire, SPI, Serial, pinMode).
// Hardware init happens in setup() via initIMUs() / initFSRs().
// IMUs Objects
IMU imu_hip  ("Hip IMU",   IMU1_address, I2C_Bus);
IMU imu_thigh("Thigh IMU", IMU2_address, I2C_Bus);

// Madgwick Filter Object (β=0.1, 100 Hz — must match sensorTask rate)
MadgwickFilter madgwick_hip  ("hip",   0.1f, 100.0f);
MadgwickFilter madgwick_thigh("thigh", 0.1f, 100.0f);

// Gait FSM Objects (dt = 10 ms — must match sensorTask rate)
GaitFSM fsm_left ("left",  0.01f);
GaitFSM fsm_right("right", 0.01f);

// Torque Controllers
// direction: flip sign if the motor is mounted inverted on one side
Controller ctrl_left ("left",  15.0f,  1.0f);
Controller ctrl_right("right", 15.0f, -1.0f);

// CAN motor controller (CAN1, 1 Mbit/s — XDrive Mini nodes: left=0, right=1)
MotorController motors(0, 1);

// FSR Objects
FSR fsr_left_heel ("Left Heel FSR",  LEFT_HEEL_FSR_PIN);
FSR fsr_left_toe  ("Left Toe FSR",   LEFT_TOE_FSR_PIN);
FSR fsr_right_heel("Right Heel FSR", RIGHT_HEEL_FSR_PIN);
FSR fsr_right_toe ("Right Toe FSR",  RIGHT_TOE_FSR_PIN);


//  Peripheral Initialization Functions
// -------------------------------------------------------------------------------------------

// Initiate Serial Monitor
static void initSerial() {
    Serial.begin(115200);
}

// Setup E-Stop Button and ISR
static void initEStop() {
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), ESTOP_ISR, FALLING);
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 0); // Set GPIO interrupt to highest priority (0)
}

// Initiate IMU Objects
static void initIMUs() {
    I2C_Bus->begin();
    imu_hip.init();
    imu_thigh.init();
}

// Initiate FSR Objects 
static void initFSRs() {
    fsr_left_heel.init();
    fsr_left_toe.init();
    fsr_right_heel.init();
    fsr_right_toe.init();
}

// Initiate SD Card Objects 
static void initSDCard() {

}

// Initiate Motor Driver Objects
// CAN1: TX=pin 22, RX=pin 23, via SN65HVD230 transceiver → XDrive Mini nodes (left=0, right=1)
static void initMotorDriver() {
    motors.init(1000000);
    motors.enable();
}

//  E-stop ISR 
// -------------------------------------------------------------------------------------------
void ESTOP_ISR() {
    estop_triggered = true;
}

// FreeRTOS Scheduled Tasks
// -------------------------------------------------------------------------------------------
// ──────────────────────────────────────────────
//  safetyTask — 1 kHz, priority 10 (highest)
//  Clamp, estop check, companion watchdog.
//  Preempts all other tasks within next 1 ms tick.
// ──────────────────────────────────────────────
static void safetyTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t led_timer = 0;

    for (;;) {
        // LED heartbeat — 1 Hz toggle so we can confirm the scheduler is running
        // independently of Serial. Visible immediately if safetyTask dispatches.
        if (++led_timer >= 500) { 
            led_timer = 0; 
            digitalToggle(LED_BUILTIN); 
    }

        // E-stop Check — disable motors once then spin forever; never return from a FreeRTOS task
        if (estop_triggered) {
            motors.disable();
            for (;;) vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

// ──────────────────────────────────────────────
//  controlTask — 200 Hz, priority 8
//  Re-reads encoders at full rate, runs AdaptiveHip,
//  sends ODrive ASCII torque commands over UART2 + UART3.
// ──────────────────────────────────────────────
static void controlTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // TODO: re-read AS5047P encoders for velocity admittance (Step 2)

        // Read shared sensor state — aligned reads are atomic on Cortex-M7
        uint8_t state_L = gait_state_left;
        uint8_t state_R = gait_state_right;
        float   phi_L   = gait_phi_left;
        float   phi_R   = gait_phi_right;
        float   gain    = assistive_gain;

        // Torque profile + clamp (Controller::compute also clamps internally)
        float tau_L = ctrl_left.compute (state_L, phi_L, gain);
        float tau_R = ctrl_right.compute(state_R, phi_R, gain);

        // CAN torque TX — single frame per axis, no stagger needed
        motors.setTorque(tau_L, tau_R);

        // Expose commands to safetyTask for watchdog clamping
        tau_cmd_L = tau_L;
        tau_cmd_R = tau_R;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
    }
}

// ──────────────────────────────────────────────
//  sensorTask — 100 Hz, priority 6
//  IMU read, ADS1115 FSR, encoder SPI, gait FSM update.
// ──────────────────────────────────────────────
static void sensorTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    I2C_Bus->begin();  // re-init LPI2C in task context; clears error state from scheduler startup

    for (;;) {
        // SPI — AS5047P ×2 (θ, ω at sensor rate)
        // TODO: assert CS_L (pin 10) low → send 0x3FFF → read 14-bit count → deassert
        // TODO: repeat for CS_R (pin 9)
        // TODO: θ_joint = (count / 16384.0) × 2π / gear_ratio (10:1)
        // TODO: ω = (θ_current - θ_prev) / 0.01s; handle ±π wrap-around

        // IMU + Madgwick Filter
        imu_hip.read(&imu_hip_accel, &imu_hip_gyro);
        madgwick_hip.update(&imu_hip_accel, &imu_hip_gyro, &imu_hip_quaternion, &imu_hip_flex_angle);

        imu_thigh.read(&imu_thigh_accel, &imu_thigh_gyro);
        madgwick_thigh.update(&imu_thigh_accel, &imu_thigh_gyro, &imu_thigh_quaternion, &imu_thigh_flex_angle);
        
        // FSRs 
        fsr_left_heel.read(&fsr_left_heel_value, &fsr_left_heel_contact);
        fsr_left_toe.read(&fsr_left_toe_value, &fsr_left_toe_contact);
        fsr_right_heel.read(&fsr_right_heel_value, &fsr_right_heel_contact);
        fsr_right_toe.read(&fsr_right_toe_value, &fsr_right_toe_contact);

        // Gait FSM update (runs after all sensor reads)
        // gyro Y-axis: sagittal-plane pitch rate (rad/s, positive = flexion)
        fsm_left.update (fsr_left_heel_contact,  fsr_left_toe_contact,  imu_hip_gyro.y);
        fsm_right.update(fsr_right_heel_contact, fsr_right_toe_contact, imu_hip_gyro.y);

        gait_state_left  = static_cast<uint8_t>(fsm_left.getState());
        gait_state_right = static_cast<uint8_t>(fsm_right.getState());
        gait_phi_left    = fsm_left.getPhi();
        gait_phi_right   = fsm_right.getPhi();
        
        imu_hip.teleplot();

        // Serial Monitor (for debugging only)

        // imu_hip.printData();
        // madgwick_hip.printData();

        // imu_thigh.printData();
        // madgwick_thigh.printData();

        // fsr_left_heel.printAnalogData();
        // fsr_left_toe.printAnalogData();
        // fsr_right_heel.printAnalogData();
        // fsr_right_toe.printAnalogData();

        // fsm_left.printData();
        // fsm_right.printData();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ──────────────────────────────────────────────
//  loggerTask — 50 Hz, priority 2 (lowest)
//  Append 64-byte record to RAM buffer.
//  Flush to SD card only when buffer is full (~1280 ms).
//  Never runs inside the control path.
// ──────────────────────────────────────────────
static void loggerTask(void* /*pvParam  s*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // TODO: append 64-byte log record to RAM ring buffer (~10 µs)
        // TODO: if (buffer >= 4096 bytes) → SD.write() flush (~5–20 ms, this task only)

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

// ──────────────────────────────────────────────
//  commsTask — 50 Hz, priority 4
//  TX 68-byte TeensyPacket to companion.
//  RX + validate 16-byte CompanionPacket from companion.
// ──────────────────────────────────────────────
static void commsTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Extra: Implementation of Rasberry Pi 
        
        // TX — pack TeensyPacket (68 bytes) and send over Serial1 at 921600 baud
        // TODO: fill sync[2]={0xAA,0xFF}, timestamp_ms, imu_L[6], imu_R[6]
        // TODO: fill encoder_L_rad, encoder_R_rad, fsr[4], gait_phase, tau_cmd_L/R, flags
        // TODO: compute XOR checksum over all preceding bytes
        // TODO: Serial1.write((uint8_t*)&pkt, sizeof(TeensyPacket))
        // RX — check for incoming CompanionPacket (16 bytes) from Raspberry Pi
        // TODO: read Serial1 into CompanionPacket buffer
        // TODO: validate sync bytes 0xBB 0xEE + XOR checksum + range checks
        // TODO: on valid packet: assistive_gain = pkt.assistive_gain
        //                        stiffness      = pkt.stiffness
        //                        profile_id     = pkt.profile_id
        //                        last_companion_ms = millis()

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}

//  Setup / FreeRTOS Scheduler Start
// -------------------------------------------------------------------------------------------
void setup() {
    // Peripheral Initialization
    initSerial();
    initEStop();
    initIMUs();
    initFSRs();
    initSDCard();
    initMotorDriver();

    // Create FreeRTOS tasks with appropriate stack sizes and priorities.
    // configMAX_PRIORITIES = 10, so valid range is 0–9 (priority 10 is out of bounds).
    xTaskCreate(safetyTask,  "safety",  512,  nullptr, 9, nullptr);
    xTaskCreate(controlTask, "control", 2048, nullptr, 7, nullptr);
    xTaskCreate(sensorTask,  "sensor",  2048, nullptr, 5, nullptr);
    // xTaskCreate(commsTask,   "comms",   1024, nullptr, 3, nullptr);
    xTaskCreate(loggerTask,  "logger",  1024, nullptr, 1, nullptr);

    vTaskStartScheduler();
}

void loop() {
    // Called by FreeRTOS idle task (via vApplicationIdleHook in FreeRTOS_TEENSY4.c).
    // yield() kicks the Teensy hardware watchdog so the device doesn't reset.
    yield();
}