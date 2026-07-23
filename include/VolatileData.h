#pragma once

#include <Arduino.h>

// Shared volatile state — extern declarations only.
// Definitions are in VolatileData.cpp.
// Including this header in multiple .cpp files is safe.

// E-Stop
// --------------------------------------------------------------------------------------------
extern volatile bool estop_triggered;   // written by GPIO ISR,    read by safetyTask

// Fall detection
// --------------------------------------------------------------------------------------------
extern volatile bool fall_detected;     // written by sensorTask (FallDetector), read by safetyTask

// IMU data 
// Struct types for shared IMU data
// --------------------------------------------------------------------------------------------
struct AccelData { 
    float x; 
    float y; 
    float z; 
};
struct GyroData  { 
    float x; 
    float y; 
    float z; 
};

// Raw IMU data — two hip IMUs (redundant, mounted on the hip) for sensor fusion
extern volatile AccelData imu_hip1_accel;
extern volatile GyroData  imu_hip1_gyro;
extern volatile AccelData imu_hip2_accel;
extern volatile GyroData  imu_hip2_gyro;

// Filtered IMU data (Madgwick output — per sensor)
// --------------------------------------------------------------------------------------------
struct QuaternionData {
    float w;
    float x;
    float y;
    float z;
};

extern volatile QuaternionData imu_hip1_quaternion;    // orientation quaternion [w, x, y, z]
extern volatile float          imu_hip1_angle_y;    // sagittal-plane hip flexion, degrees
extern volatile QuaternionData imu_hip2_quaternion;
extern volatile float          imu_hip2_angle_y;    // sagittal-plane hip flexion, degrees

// Fused pelvis estimate — basic sensor fusion (average of the two pelvis-mounted IMUs).
// NOTE: the IMUs sit on the sacrum/pelvis, so these measure PELVIS pitch in the world frame,
// NOT hip joint flexion (that is the encoder / motor_angle). Downstream consumers should use
// these for trunk lean / world reference, and the encoder for the actual hip joint angle.
// --------------------------------------------------------------------------------------------
extern volatile float pelvis_pitch_y;      // fused sagittal-plane pelvis pitch, degrees
extern volatile float pelvis_pitch_rate_y; // fused sagittal-plane pelvis pitch rate, rad/s

// FSR data
// --------------------------------------------------------------------------------------------
// Raw FSR values (0–255)
extern volatile uint8_t fsr_left_heel_value; 
extern volatile uint8_t fsr_left_toe_value;  
extern volatile uint8_t fsr_right_heel_value; 
extern volatile uint8_t fsr_right_toe_value;  

// Contact states
extern volatile bool fsr_left_heel_contact; 
extern volatile bool fsr_left_toe_contact;  
extern volatile bool fsr_right_heel_contact; 
extern volatile bool fsr_right_toe_contact;  

// Gait phase (written by sensorTask via GaitFSM; read by controlTask / logger / comms)
// --------------------------------------------------------------------------------------------
extern volatile float   gait_phase_L;    // continuous stride phase [0,1], 0 = left heel strike
extern volatile float   gait_phase_R;    // continuous stride phase [0,1], 0 = right heel strike
extern volatile uint8_t gait_state_L;    // GaitState (0=LOADING,1=MID_STANCE,2=TERMINAL_STANCE,3=SWING)
extern volatile uint8_t gait_state_R;

// Motor control
// --------------------------------------------------------------------------------------------
extern volatile float tau_cmd_L;         // Nm at the joint — sent via MotorCAN::setTorqueNm()
extern volatile float tau_cmd_R;         // Nm at the joint

// Motor state feedback (written by controlTask; joint side — MotorCAN::read() divides
// the motor-shaft encoder values by the 15:1 gear ratio)
extern volatile float motor_angle_L;     // rad, joint side (left XDrive Mini)
extern volatile float motor_vel_L;       // rad/s, joint side
extern volatile float motor_angle_R;     // rad, joint side (right XDrive Mini)
extern volatile float motor_vel_R;       // rad/s, joint side
