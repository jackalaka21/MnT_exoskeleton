#include "VolatileData.h"

// The one definition of each shared global. Everything else sees the externs in VolatileData.h.

// Fall detection
// --------------------------------------------------------------------------------------------
volatile bool fall_detected = false;

// IMU data
// --------------------------------------------------------------------------------------------
volatile AccelData imu_hip1_accel = {};
volatile GyroData  imu_hip1_gyro  = {};
volatile AccelData imu_hip2_accel = {};
volatile GyroData  imu_hip2_gyro  = {};

// Madgwick output
volatile QuaternionData imu_hip1_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float          imu_hip1_angle_y    = 0.0f;
volatile QuaternionData imu_hip2_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float          imu_hip2_angle_y    = 0.0f;

// Fused pelvis estimate
volatile float pelvis_pitch_y      = 0.0f;
volatile float pelvis_pitch_rate_y = 0.0f;

// FSR data
// --------------------------------------------------------------------------------------------
volatile uint8_t fsr_left_heel_value  = 0;
volatile uint8_t fsr_left_toe_value   = 0;
volatile uint8_t fsr_right_heel_value = 0;
volatile uint8_t fsr_right_toe_value  = 0;

volatile bool fsr_left_heel_contact  = false;
volatile bool fsr_left_toe_contact   = false;
volatile bool fsr_right_heel_contact = false;
volatile bool fsr_right_toe_contact  = false;

// Gait
// --------------------------------------------------------------------------------------------
volatile float   gait_phase_L = 0.0f;
volatile float   gait_phase_R = 0.0f;
volatile uint8_t gait_state_L = 1;   // GaitState::MID_STANCE
volatile uint8_t gait_state_R = 1;

// Boots true (both feet planted = safe default, zero assist)
volatile bool standing_detected = true;

// Motor
// --------------------------------------------------------------------------------------------
volatile float tau_cmd_L     = 0.0f;
volatile float tau_cmd_R     = 0.0f;
volatile float motor_angle_L = 0.0f;
volatile float motor_vel_L   = 0.0f;
volatile float motor_angle_R = 0.0f;
volatile float motor_vel_R   = 0.0f;
