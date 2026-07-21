#pragma once

#include <Arduino.h>

// Gait phase estimator — one instance per leg.
//
// Produces two outputs every sensor cycle:
//   1. a discrete GaitState (STANCE → PRE_SWING → SWING), driven entirely by the foot FSRs; and
//   2. a continuous phase in [0.0, 1.0] spanning the WHOLE stride (0 = heel strike),
//      normalised against the measured stride period so it stays in sync as cadence
//      changes — unlike a fixed-period timer, which drifts the moment walking speed does.
//
// State diagram (driven entirely by the two foot FSRs — heel and toe contact):
//
//                          heel off + toe on
//   ┌────────┐ ──────────────────────────────────────────────► ┌───────────┐
//   │ STANCE │                                                  │ PRE_SWING  │
//   └────────┘ ◄──────────────── heel re-plant ──────────────── └───────────┘
//       ▲                                                            │ both FSRs clear
//       │              heel strike (phase→0, stride measured)        ▼
//       │                                                        ┌───────┐
//       └──────────────────────────────────────────────────────  │ SWING │
//                                                                 └───────┘
//
// FSR-only: the heel/toe contact flags are the sole input. The push-off transition used to also
// gate on hip-joint velocity (motor_vel_*) to reject slow weight-shifts, but that made the FSM
// impossible to exercise without real thigh motion, so it was removed — contact alone drives it.

enum class GaitState : uint8_t {
    STANCE    = 0,   // heel and/or toe loaded — foot on the ground, bearing weight
    PRE_SWING = 1,   // heel lifted, toe still down — pushing off, about to leave the ground
    SWING     = 2,   // both FSRs clear — foot airborne
};

class GaitFSM {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // name : label prefix used in Serial / Teleplot output   (e.g. "left", "right")
        // dt   : sample period in seconds — MUST match the rate update() is called at (0.01 = 100 Hz)
        GaitFSM(String name, float dt = 0.01f);

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Call once per sensor cycle, after the FSR reads.
        //   heel_contact / toe_contact : debounced FSR contact flags for THIS leg
        // FSR-only: the foot contact flags are the sole input driving the state machine.
        void update(bool heel_contact, bool toe_contact);

        // Continuous gait phase over the full stride: 0.0 at heel strike → 1.0 at the next.
        // This is the primary signal for the assistive controller.
        float phase() const { return _phase; }

        GaitState state() const { return _state; }

        // Measured stride period in seconds (EMA-smoothed). Falls back to NOMINAL_STRIDE_S
        // until the first full stride has been observed.
        float stridePeriod() const { return _stride_period; }

        // Edge events — true for exactly one cycle after they occur.
        bool heelStrike() const { return _heel_strike; }   // SWING → STANCE
        bool toeOff()     const { return _toe_off; }        // PRE_SWING → SWING

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printData();

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String    _name;
        float     _dt;

        GaitState _state         = GaitState::STANCE;
        float     _phase         = 0.0f;    // [0, 1] over the stride
        float     _t_in_stride   = 0.0f;    // seconds elapsed since the last heel strike
        float     _stride_period = 0.0f;    // EMA-smoothed stride time, seconds

        bool      _heel_strike   = false;   // one-cycle event flags
        bool      _toe_off       = false;

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // Register a completed stride: update the smoothed period and restart the phase clock.
        void _closeStride();

        // Tuning constants live in Config.h → Config::Gait (stride window, EMA).
};
