#pragma once

#include <Arduino.h>

// Shared state between tasks — declarations only, defined in VolatileData.cpp.
// Safe to include from any number of .cpp files.

// Fall detection
// --------------------------------------------------------------------------------------------
extern volatile bool fall_detected;     // set by sensorTask, read by safetyTask

// IMU data
// --------------------------------------------------------------------------------------------
struct AccelData {
    float x;
    float y;
    float z;
};
struct GyroData {
    float x;
    float y;
    float z;
};

// Raw data from the two hip IMUs (redundant pair, fused below)
extern volatile AccelData imu_hip1_accel;
extern volatile GyroData  imu_hip1_gyro;
extern volatile AccelData imu_hip2_accel;
extern volatile GyroData  imu_hip2_gyro;

// Madgwick output, per sensor
// --------------------------------------------------------------------------------------------
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
};

extern volatile QuaternionData imu_hip1_quaternion;
extern volatile float          imu_hip1_angle_y;   // sagittal angle, degrees
extern volatile QuaternionData imu_hip2_quaternion;
extern volatile float          imu_hip2_angle_y;   // sagittal angle, degrees

// Fused pelvis estimate — average of the two IMUs.
// They sit on the sacrum, so this is PELVIS pitch in the world frame, not hip joint flexion.
// Use it for trunk lean; use the encoder (motor_angle) for the actual joint angle.
// --------------------------------------------------------------------------------------------
extern volatile float pelvis_pitch_y;      // degrees
extern volatile float pelvis_pitch_rate_y; // rad/s

// FSR data
// --------------------------------------------------------------------------------------------
// Filtered values (0–255)
extern volatile uint8_t fsr_left_heel_value;
extern volatile uint8_t fsr_left_toe_value;
extern volatile uint8_t fsr_right_heel_value;
extern volatile uint8_t fsr_right_toe_value;

// Contact states
extern volatile bool fsr_left_heel_contact;
extern volatile bool fsr_left_toe_contact;
extern volatile bool fsr_right_heel_contact;
extern volatile bool fsr_right_toe_contact;

// Gait — written by sensorTask via GaitFSM, read by controlTask / monitor
// --------------------------------------------------------------------------------------------
extern volatile float   gait_phase_L;    // stride phase [0,1], 0 = heel strike
extern volatile float   gait_phase_R;
extern volatile uint8_t gait_state_L;    // GaitState (0=LOADING, 1=MID_STANCE, 2=TERMINAL_STANCE, 3=SWING)
extern volatile uint8_t gait_state_R;

// True once both feet have stayed loaded past STANDING_DOUBLE_SUPPORT_S — zeroes assist on both legs.
extern volatile bool standing_detected;

// Motor command
// --------------------------------------------------------------------------------------------
extern volatile float tau_cmd_L;         // Nm at the joint, sent via MotorCAN::setTorqueNm()
extern volatile float tau_cmd_R;

// Motor feedback, joint side (MotorCAN::read() divides out the gear ratio)
extern volatile float motor_angle_L;     // rad
extern volatile float motor_vel_L;       // rad/s
extern volatile float motor_angle_R;
extern volatile float motor_vel_R;
