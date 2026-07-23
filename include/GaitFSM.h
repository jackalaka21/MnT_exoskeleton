#pragma once

#include <Arduino.h>

// Gait phase estimator — one instance per leg.
//
// Produces two outputs every sensor cycle:
//   1. a discrete GaitState — the four canonical gait phases, decoded directly from the two
//      foot FSRs (heel + toe contact); and
//   2. a continuous phase in [0.0, 1.0] spanning the WHOLE stride (0 = heel strike),
//      normalised against the measured stride period so it stays in sync as cadence
//      changes — unlike a fixed-period timer, which drifts the moment walking speed does.
//
// The two FSRs give four (heel, toe) contact combinations, one per phase, so the discrete state
// is a direct decode of foot contact:
//
//   heel toe   phase
//   ●    ○     LOADING          initial contact / weight acceptance (heel strike — phase 0)
//   ●    ●     MID_STANCE       foot flat, body passes over the planted foot
//   ○    ●     TERMINAL_STANCE  heel lifted, pushing off
//   ○    ○     SWING            foot airborne
//
// The stride's phase-0 reference is initial ground contact after swing (SWING → any stance);
// toe-off is the return to SWING. FSR-only: the heel/toe contact flags are the sole input. The
// push-off transition used to also gate on hip-joint velocity (motor_vel_*) to reject slow
// weight-shifts, but that made the FSM impossible to exercise without real thigh motion, so it
// was removed — contact alone drives it.

enum class GaitState : uint8_t {
    LOADING         = 0,   // heel on,  toe off — initial contact / weight acceptance
    MID_STANCE      = 1,   // heel on,  toe on  — foot flat, body passing over the foot
    TERMINAL_STANCE = 2,   // heel off, toe on  — heel lifted, pushing off
    SWING           = 3,   // heel off, toe off — foot airborne
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
        bool heelStrike() const { return _heel_strike; }   // SWING → stance (initial contact)
        bool toeOff()     const { return _toe_off; }        // stance → SWING (foot airborne)

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printData();

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String    _name;
        float     _dt;

        GaitState _state         = GaitState::MID_STANCE;   // both FSRs loaded — safe standing default
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
