#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include "Config.h"
#include "Pin.h"
#include "VolatileData.h"

#include "IMU.h"
#include "FSR.h"
#include "MadgwickFilter.h"
#include "MotorCAN.h"
#include "Controller.h"
#include "FallDetector.h"

// Global Peripheral Objects
// -------------------------------------------------------------------------------------------

// Two hip IMUs (redundant pair, fused in sensorTask)
IMU imu_hip1("Hip IMU 1", IMU1_address, I2C_Bus);
IMU imu_hip2("Hip IMU 2", IMU2_address, I2C_Bus);

// Madgwick filters — sampleHz must match the sensorTask rate
MadgwickFilter madgwick_hip1("hip1", Config::Imu::MADGWICK_BETA, Config::Rates::SENSOR_HZ);
MadgwickFilter madgwick_hip2("hip2", Config::Imu::MADGWICK_BETA, Config::Rates::SENSOR_HZ);

// FSRs — last arg is the per-sensor gain, so each of the four can be matched independently
FSR fsr_left_heel ("Left Heel FSR",  LEFT_HEEL_FSR_PIN,  Config::FSR::THRESHOLD,     Config::FSR::EMA_ALPHA, Config::FSR::LEFT_HEEL_GAIN);
FSR fsr_left_toe  ("Left Toe FSR",   LEFT_TOE_FSR_PIN,   Config::FSR::TOE_THRESHOLD, Config::FSR::EMA_ALPHA, Config::FSR::LEFT_TOE_GAIN);
FSR fsr_right_heel("Right Heel FSR", RIGHT_HEEL_FSR_PIN, Config::FSR::THRESHOLD,     Config::FSR::EMA_ALPHA, Config::FSR::RIGHT_HEEL_GAIN);
FSR fsr_right_toe ("Right Toe FSR",  RIGHT_TOE_FSR_PIN,  Config::FSR::TOE_THRESHOLD, Config::FSR::EMA_ALPHA, Config::FSR::RIGHT_TOE_GAIN);

// CAN bus to the MKS XDrive Mini drives (SimpleFOC CANCommander protocol)
MotorCAN motor_can("Motor CAN", MOTOR_NODE_L, MOTOR_NODE_R);

// Gait estimation + torque commands for both legs
Controller controller(motor_can);

// Watches the fused pelvis pitch for a topple
FallDetector fall_detector(Config::Rates::SENSOR_DT);

// Lets safetyTask stop the control loop before killing the motors
static TaskHandle_t control_task_handle = nullptr;


// Peripheral Initialization
// -------------------------------------------------------------------------------------------
static void initSerial() {
    Serial.begin(115200);
}

static void initIMUs() {
    I2C_Bus->begin();
    imu_hip1.init();
    imu_hip2.init();
}

static void initFSRs() {
    fsr_left_heel.init();
    fsr_left_toe.init();
    fsr_right_heel.init();
    fsr_right_toe.init();
}

// Bring up CAN2, then both drives. All or nothing: if either drive is missing, startup() leaves
// both disabled and armed() stays false.
static void initMotorDriver() {
    motor_can.init(MOTOR_CAN_BAUD);
    motor_can.startup();
}


// FreeRTOS Tasks
// -------------------------------------------------------------------------------------------
// ──────────────────────────────────────────────
//  safetyTask — 1 kHz, priority 9 (highest)
//  Watches for a detected fall; preempts everything else within a tick.
// ──────────────────────────────────────────────

// Drop into the safe state and stay there. Stops the control loop first so it can't send another
// torque command, then disables the drives. Never returns — a FreeRTOS task must not fall off the
// end, so it spins with a fast LED blink.
static void enterSafeState() {
    if (control_task_handle != nullptr) vTaskSuspend(control_task_handle);
    motor_can.disableAll();
    for (;;) {
        digitalToggle(LED_BUILTIN);        // fast blink = latched fault
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void safetyTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t led_timer = 0;

    for (;;) {
        // 1 Hz heartbeat — confirms the scheduler is running without needing Serial.
        if (++led_timer >= 500) {
            led_timer = 0;
            digitalToggle(LED_BUILTIN);
        }

        // Latches the exo down until it's power-cycled (enterSafeState never returns).
        if (fall_detected) {
            enterSafeState();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}

// ──────────────────────────────────────────────
//  controlTask — 100 Hz, priority 7
//  Reads motor feedback and commands each leg's assistive torque.
// ──────────────────────────────────────────────
static void controlTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        controller.applyAssistiveTorque();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ──────────────────────────────────────────────
//  sensorTask — 100 Hz, priority 5
//  IMUs + Madgwick, fall detection, FSRs, gait update.
// ──────────────────────────────────────────────
static void sensorTask(void* /*pvParams*/) {
    TickType_t last_wake = xTaskGetTickCount();

    I2C_Bus->begin();   // re-init LPI2C in task context; clears errors left by scheduler startup

    for (;;) {
        // IMUs — the two run independently through their own filters
        imu_hip1.read(&imu_hip1_accel, &imu_hip1_gyro);
        madgwick_hip1.update(&imu_hip1_accel, &imu_hip1_gyro, &imu_hip1_quaternion, &imu_hip1_angle_y);

        imu_hip2.read(&imu_hip2_accel, &imu_hip2_gyro);
        madgwick_hip2.update(&imu_hip2_accel, &imu_hip2_gyro, &imu_hip2_quaternion, &imu_hip2_angle_y);

        // Both IMUs sit on the sacrum measuring the same pelvis pitch, so averaging them halves
        // the uncorrelated noise.
        pelvis_pitch_y      = 0.5f * (imu_hip1_angle_y + imu_hip2_angle_y);
        pelvis_pitch_rate_y = 0.5f * (imu_hip1_gyro.y  + imu_hip2_gyro.y);

        // A latched fall becomes a safe-state shutdown on safetyTask's next tick.
        if (fall_detector.update(pelvis_pitch_y)) fall_detected = true;

        // FSRs
        fsr_left_heel.read(&fsr_left_heel_value, &fsr_left_heel_contact);
        fsr_left_toe.read(&fsr_left_toe_value, &fsr_left_toe_contact);
        fsr_right_heel.read(&fsr_right_heel_value, &fsr_right_heel_contact);
        fsr_right_toe.read(&fsr_right_toe_value, &fsr_right_toe_contact);

        // Gait phase for both legs
        controller.updateGaitPhase();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

// ──────────────────────────────────────────────
//  monitorTask — 10 Hz, priority 2
//  Debug output: an IMU line, a CAN line, then one line per leg so the two sides line up.
// ──────────────────────────────────────────────

// One leg: gait, motor, and both foot FSRs.
static void printLeg(const char* label, uint8_t gait_state, float phase,
                     float angle, float vel, float tau,
                     uint8_t heel_val, bool heel_contact,
                     uint8_t toe_val,  bool toe_contact) {
    // Padded so the LEFT and RIGHT columns line up.
    static const char* const GAIT_NAMES[] = { "LOADING   ", "MIDSTANCE ", "TERMSTANCE", "SWING     " };

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
        // Serial.print only — Serial.printf goes through newlib vdprintf, which isn't safe from
        // tasks on this port.
        Serial.println("=================== EXO STATE ===================");

        // IMUs and the fused pelvis pitch
        Serial.print("IMU   | hip1 ");   Serial.print(imu_hip1_angle_y, 1);
        Serial.print(" deg  hip2 ");     Serial.print(imu_hip2_angle_y, 1);
        Serial.print(" deg | pelvis ");  Serial.print(pelvis_pitch_y, 1);
        Serial.print(" deg  rate ");     Serial.print(pelvis_pitch_rate_y, 2);
        Serial.print(" rad/s | FALL ");  Serial.print(fall_detected ? "1" : "0");
        Serial.print(" | STANDING ");    Serial.println(standing_detected ? "1" : "0");

        // ms since each drive last answered a read. Climbing = it has stopped responding; small
        // but with angle/vel stuck at 0 = it answers, but its position sensor isn't configured.
        uint32_t now = millis();
        Serial.print("CAN   | node ");   Serial.print(MOTOR_NODE_L);
        Serial.print(" (L) resp age ");  Serial.print(now - motor_can.lastResponseMs(MOTOR_NODE_L));
        Serial.print(" ms | node ");     Serial.print(MOTOR_NODE_R);
        Serial.print(" (R) resp age ");  Serial.print(now - motor_can.lastResponseMs(MOTOR_NODE_R));
        Serial.println(" ms");

        printLeg("LEFT ", gait_state_L, gait_phase_L, motor_angle_L, motor_vel_L, tau_cmd_L,
                 fsr_left_heel_value,  fsr_left_heel_contact,  fsr_left_toe_value,  fsr_left_toe_contact);
        printLeg("RIGHT", gait_state_R, gait_phase_R, motor_angle_R, motor_vel_R, tau_cmd_R,
                 fsr_right_heel_value, fsr_right_heel_contact, fsr_right_toe_value, fsr_right_toe_contact);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

// Setup / Scheduler Start
// -------------------------------------------------------------------------------------------
void setup() {
    initSerial();
    initIMUs();
    initFSRs();
    initMotorDriver();

    // configMAX_PRIORITIES = 10, so priorities run 0–9.
    xTaskCreate(safetyTask,  "safety",  512,  nullptr, 9, nullptr);
    xTaskCreate(controlTask, "control", 2048, nullptr, 7, &control_task_handle);
    xTaskCreate(sensorTask,  "sensor",  2048, nullptr, 5, nullptr);
    xTaskCreate(monitorTask, "monitor", 1024, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void loop() {
    // Runs from the FreeRTOS idle hook. yield() kicks the hardware watchdog so we don't reset.
    yield();
}
