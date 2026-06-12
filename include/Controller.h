#pragma once

#include <Arduino.h>

// Torque profile controller — one instance per leg.
//
// Profile shape (swing-phase assistance only):
//
//   τ(φ) = peak_Nm × gain × sin(π × φ)
//
//   φ=0.0  ──── φ=0.5  ──── φ=1.0
//     0      peak_Nm      0
//
// Zero torque is commanded during STANCE and PUSH_OFF states.
// The profile can be extended to a full lookup table once biomechanics
// data are available — swap compute() internals without changing the interface.

class Controller {
public:
    // name         : label prefix for debug output  (e.g. "left", "right")
    // peak_torque  : assistive torque amplitude in Nm
    // direction    : +1.0 if positive motor torque = hip flexion,
    //                -1.0 if motor is mounted inverted on this side
    Controller(String name, float peak_torque_nm = 15.0f, float direction = 1.0f);

    // Compute assistive torque command in Nm.
    // gait_state  : 0=STANCE, 1=PUSH_OFF, 2=SWING  (matches GaitState enum)
    // phi         : normalised swing phase [0.0–1.0]
    // gain        : runtime assistive-gain scalar (from assistive_gain volatile, 0–1)
    float compute(uint8_t gait_state, float phi, float gain = 1.0f);

    static constexpr float MAX_TORQUE_NM = 14.0f;  // hard clamp; firmware TORQUE_OUT_MAX ≈ 14.49 Nm (GEAR_EFF=0.6)

    // Serial Monitor (for debugging only)
    void printData(float tau) const;


private:
    String _name;
    float  _peak_Nm;
    float  _dir;

    static constexpr uint8_t GAIT_SWING = 2;  // GaitState::SWING cast to uint8_t
};
