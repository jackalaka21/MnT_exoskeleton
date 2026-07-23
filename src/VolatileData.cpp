#include "VolatileData.h"

// Actual definitions — only compiled once.
// All other files see only the extern declarations in VolatileData.h.

// E-Stop data
// --------------------------------------------------------------------------------------------
volatile bool estop_triggered   = false;

// Fall detection
// --------------------------------------------------------------------------------------------
volatile bool fall_detected     = false;

// IMU data
// --------------------------------------------------------------------------------------------
// Raw IMU data — two hip IMUs
volatile AccelData imu_hip1_accel  = {};
volatile GyroData  imu_hip1_gyro   = {};
volatile AccelData imu_hip2_accel  = {};
volatile GyroData  imu_hip2_gyro   = {};

// Filtered IMU data (Madgwick output — per sensor)
volatile QuaternionData imu_hip1_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float          imu_hip1_angle_y = 0.0f;
volatile QuaternionData imu_hip2_quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float          imu_hip2_angle_y = 0.0f;

// Fused pelvis estimate (average of the two pelvis-mounted IMUs — pelvis pitch, not hip flexion)
volatile float pelvis_pitch_y      = 0.0f;
volatile float pelvis_pitch_rate_y = 0.0f;

// FSR data
// --------------------------------------------------------------------------------------------
// Raw FSR values (0–255)
volatile uint8_t fsr_left_heel_value = {}; 
volatile uint8_t fsr_left_toe_value = {};
volatile uint8_t fsr_right_heel_value = {};
volatile uint8_t fsr_right_toe_value = {};

// Contact states
extern volatile bool fsr_left_heel_contact = {}; 
extern volatile bool fsr_left_toe_contact = {};  
extern volatile bool fsr_right_heel_contact = {}; 
extern volatile bool fsr_right_toe_contact = {}; 

// Gait phase
// --------------------------------------------------------------------------------------------
volatile float   gait_phase_L = 0.0f;
volatile float   gait_phase_R = 0.0f;
volatile uint8_t gait_state_L = 1;   // GaitState::MID_STANCE
volatile uint8_t gait_state_R = 1;

// Motor control
// --------------------------------------------------------------------------------------------
volatile float tau_cmd_L = 0.0f;
volatile float tau_cmd_R = 0.0f;
volatile float motor_angle_L = 0.0f;
volatile float motor_vel_L   = 0.0f;
volatile float motor_angle_R = 0.0f;
volatile float motor_vel_R   = 0.0f;
