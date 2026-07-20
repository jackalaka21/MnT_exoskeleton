#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include "Pin.h"
#include "VolatileData.h"

#include "IMU.h"
#include "FSR.h"
#include "MadgwickFilter.h"
#include "MotorCAN.h"
#include "Controller.h"

// Forward declarations 
// -------------------------------------------------------------------------------------------
void ESTOP_ISR();

// Global Peripherals Objects
// -------------------------------------------------------------------------------------------

// IMUs Objects — two hip IMUs (redundant, fused in sensorTask)
IMU imu_hip1("Hip IMU 1", IMU1_address, I2C_Bus);
IMU imu_hip2("Hip IMU 2", IMU2_address, I2C_Bus);

// Madgwick Filter Object (β=0.1, 100 Hz — must match sensorTask rate)
MadgwickFilter madgwick_hip1("hip1", 0.1f, 100.0f);
MadgwickFilter madgwick_hip2("hip2", 0.1f, 100.0f);

// FSR Objects
FSR fsr_left_heel ("Left Heel FSR",  LEFT_HEEL_FSR_PIN);
FSR fsr_left_toe  ("Left Toe FSR",   LEFT_TOE_FSR_PIN);
FSR fsr_right_heel("Right Heel FSR", RIGHT_HEEL_FSR_PIN);
FSR fsr_right_toe ("Right Toe FSR",  RIGHT_TOE_FSR_PIN);

// Motor CAN bus — SimpleFOC CANCommander protocol to the MKS XDrive Mini drives
MotorCAN motor_can("Motor CAN", MOTOR_NODE_L, MOTOR_NODE_R);

// Assistive gait controller — owns the per-leg gait estimation and torque command
Controller controller(motor_can);

// Handle so safetyTask can suspend the control loop before disabling the motors
static TaskHandle_t control_task_handle = nullptr;


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
    imu_hip1.init();
    imu_hip2.init();
}

// Initiate FSR Objects 
static void initFSRs() {
    fsr_left_heel.init();
    fsr_left_toe.init();
    fsr_right_heel.init();
    fsr_right_toe.init();
}

// Initiate Motor CAN bus — bring up CAN2, then configure both drives. All-or-nothing: if either
// drive is missing, MotorCAN::startup() leaves both disabled (armed() stays false).
static void initMotorDriver() {
    motor_can.init(MOTOR_CAN_BAUD);
    motor_can.startup();
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
//  Clamp and estop check.
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

        // E-stop Check — spin forever; never return from a FreeRTOS task
        if (estop_triggered) {
            // Stop the control loop first so it can't send another torque command,
            // then zero the target and disable the drives.
            if (control_task_handle != nullptr) vTaskSuspend(control_task_handle);
            motor_can.disableAll();
            for (;;) vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

// ──────────────────────────────────────────────
//  controlTask — 100 Hz, priority 7
//  Reads motor feedback and commands each leg's assistive torque (via the Controller).
// ──────────────────────────────────────────────
static void controlTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        controller.applyAssistiveTorque();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
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

        // IMU + Madgwick Filter — two hip IMUs run independently
        imu_hip1.read(&imu_hip1_accel, &imu_hip1_gyro);
        madgwick_hip1.update(&imu_hip1_accel, &imu_hip1_gyro, &imu_hip1_quaternion, &imu_hip1_angle_y);

        imu_hip2.read(&imu_hip2_accel, &imu_hip2_gyro);
        madgwick_hip2.update(&imu_hip2_accel, &imu_hip2_gyro, &imu_hip2_quaternion, &imu_hip2_angle_y);

        // Basic sensor fusion — average the two redundant pelvis IMUs (both on the sacrum,
        // measuring the same pelvis pitch), so averaging halves uncorrelated noise.
        pelvis_pitch_y      = 0.5f * (imu_hip1_angle_y + imu_hip2_angle_y);
        pelvis_pitch_rate_y = 0.5f * (imu_hip1_gyro.y  + imu_hip2_gyro.y);

        // FSRs
        fsr_left_heel.read(&fsr_left_heel_value, &fsr_left_heel_contact);
        fsr_left_toe.read(&fsr_left_toe_value, &fsr_left_toe_contact);
        fsr_right_heel.read(&fsr_right_heel_value, &fsr_right_heel_contact);
        fsr_right_toe.read(&fsr_right_toe_value, &fsr_right_toe_contact);

        // Gait phase — the controller advances each leg's FSR-driven gait estimate.
        controller.updateGaitPhase();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ──────────────────────────────────────────────
//  monitorTask — 10 Hz, priority 2
//  Serial monitor for debugging. One block per cycle: an IMU line, then a LEFT and a RIGHT
//  leg chunk, each grouping that leg's gait / motor / foot-FSR state so the two sides are
//  easy to compare at a glance.
// ──────────────────────────────────────────────

// Print one leg's chunk: gait state/phase, motor angle/vel/commanded torque, and both foot FSRs.
static void printLeg(const char* label, uint8_t gait_state, float phase,
                     float angle, float vel, float tau,
                     uint8_t heel_val, bool heel_contact,
                     uint8_t toe_val,  bool toe_contact) {
    // Padded to a fixed width so the columns line up between the LEFT and RIGHT lines.
    static const char* const GAIT_NAMES[] = { "STANCE  ", "PRESWING", "SWING   " };

    Serial.print(label);
    Serial.print(" | Gait ");   Serial.print(GAIT_NAMES[gait_state]);
    Serial.print(" ph ");       Serial.print(phase, 2);
    Serial.print(" | Motor ang ");  Serial.print(angle, 3);
    Serial.print(" vel ");      Serial.print(vel, 3);
    Serial.print(" tau ");      Serial.print(tau, 2);
    Serial.print(" Nm | FSR heel "); Serial.print((int)heel_val);
    Serial.print("/");          Serial.print(heel_contact);
    Serial.print(" toe ");      Serial.print((int)toe_val);
    Serial.print("/");          Serial.println(toe_contact);
}

static void monitorTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // Plain Serial.print only — Serial.printf goes through newlib vdprintf,
        // which is not safe from FreeRTOS tasks with this port (no newlib reentrancy).
        Serial.println("=================== EXO STATE ===================");

        // IMU chunk — the two hip IMUs and their fused pelvis pitch
        Serial.print("IMU   | hip1 ");   Serial.print(imu_hip1_angle_y, 1);
        Serial.print(" deg  hip2 ");     Serial.print(imu_hip2_angle_y, 1);
        Serial.print(" deg | pelvis ");  Serial.print(pelvis_pitch_y, 1);
        Serial.print(" deg  rate ");     Serial.print(pelvis_pitch_rate_y, 2);
        Serial.println(" rad/s");

        // One chunk per leg, aligned for easy left/right comparison
        printLeg("LEFT ", gait_state_L, gait_phase_L, motor_angle_L, motor_vel_L, tau_cmd_L,
                 fsr_left_heel_value,  fsr_left_heel_contact,  fsr_left_toe_value,  fsr_left_toe_contact);
        printLeg("RIGHT", gait_state_R, gait_phase_R, motor_angle_R, motor_vel_R, tau_cmd_R,
                 fsr_right_heel_value, fsr_right_heel_contact, fsr_right_toe_value, fsr_right_toe_contact);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
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
    initMotorDriver();

    // Create FreeRTOS tasks with appropriate stack sizes and priorities.
    // configMAX_PRIORITIES = 10, so valid range is 0–9 (priority 10 is out of bounds).
    xTaskCreate(safetyTask,  "safety",  512,  nullptr, 9, nullptr);
    xTaskCreate(controlTask, "control", 2048, nullptr, 7, &control_task_handle);
    xTaskCreate(sensorTask,  "sensor",  2048, nullptr, 5, nullptr);
    xTaskCreate(monitorTask, "monitor", 1024, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void loop() {
    // Called by FreeRTOS idle task (via vApplicationIdleHook in FreeRTOS_TEENSY4.c).
    // yield() kicks the Teensy hardware watchdog so the device doesn't reset.
    yield();
}