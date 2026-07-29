#pragma once

#include <Arduino.h>

#include "Config.h"

// Gait phase estimator — one per leg. FSR contact is the only input.
//
// Outputs each sensor cycle:
//   1. a discrete GaitState, decoded straight from the heel/toe contact flags;
//   2. a continuous phase in [0, 1] across the stride (0 = heel strike), normalised against the
//      measured stride period so it tracks cadence instead of drifting like a fixed timer.
//
// Two FSRs give four contact combinations, one per phase:
//
//   heel toe   state
//   ●    ○     LOADING          heel strike / weight acceptance  (phase 0)
//   ●    ●     MID_STANCE       foot flat, body passing over
//   ○    ●     TERMINAL_STANCE  heel lifted, pushing off
//   ○    ○     SWING            airborne
//
// Phase 0 is the first ground contact after swing; toe-off is the return to SWING. Push-off used
// to also gate on hip velocity, but that made the FSM untestable without real thigh motion.

enum class GaitState : uint8_t {
    LOADING         = 0,   // heel on,  toe off
    MID_STANCE      = 1,   // heel on,  toe on
    TERMINAL_STANCE = 2,   // heel off, toe on
    SWING           = 3,   // heel off, toe off
};

class GaitFSM {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // name : label for Serial / Teleplot output ("left", "right")
        // dt   : sample period in seconds — must match how often update() is called
        GaitFSM(String name, float dt = 0.01f);

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Call once per sensor cycle with this leg's debounced contact flags.
        void update(bool heel_contact, bool toe_contact);

        // Stride phase: 0.0 at heel strike → 1.0 at the next one.
        float phase() const { return _phase; }

        GaitState state() const { return _state; }

        // How far through the current swing we are, estimated from cadence: time since toe-off
        // over the expected swing duration. ~0.5–0.6 at peak flexion, →1.0 at the estimated heel
        // strike. Only meaningful in SWING, and only an estimate — the FSRs are blind mid-air.
        // No longer drives torque (the swing push-down was removed); kept for logging.
        float swingProgress() const {
            float expected = Config::Gait::SWING_FRACTION * _stride_period;
            return (expected > 1e-3f) ? (_t_in_swing / expected) : 0.0f;
        }

        // Measured stride period (EMA), seconds. NOMINAL_STRIDE_S until the first full stride.
        float stridePeriod() const { return _stride_period; }

        // Edge events — true for exactly one cycle.
        bool heelStrike() const { return _heel_strike; }   // SWING → stance
        bool toeOff()     const { return _toe_off; }       // stance → SWING

        // True while the state sequence looks like a real stride. Set by a heel-first contact,
        // cleared by the one fault signature: TERMINAL_STANCE straight out of SWING, i.e. a
        // toe-only contact from mid-air with no heel load first. A fast heel-to-toe roll can skip
        // MID_STANCE, so LOADING → TERMINAL_STANCE is legal and stays synced.
        // Safety: the controller must withhold assist while this is false, so a faulty FSR can't
        // command torque. Starts false and re-arms on the next clean heel strike.
        bool synced() const { return _synced; }

        // True while gait edge events keep arriving; false once nothing has happened for
        // IDLE_TIMEOUT_STRIDES stride periods — wearer stopped, foot in the air, or bench.
        // Safety: the controller must command zero assist when this is false, otherwise the
        // free-running phase clock keeps issuing torque into an unloaded joint.
        bool walking() const {
            return _t_since_event < Config::Gait::IDLE_TIMEOUT_STRIDES * _stride_period;
        }

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printData();

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String    _name;
        float     _dt;

        GaitState _state         = GaitState::MID_STANCE;   // both feet loaded = safe default
        float     _phase         = 0.0f;    // [0, 1] over the stride
        float     _t_in_stride   = 0.0f;    // s since the last heel strike
        float     _t_in_swing    = 0.0f;    // s since the last toe-off
        float     _t_since_event = 1e6f;    // s since the last edge event (starts idle)
        float     _stride_period = 0.0f;    // EMA stride time, s

        bool      _heel_strike   = false;   // one-cycle event flags
        bool      _toe_off       = false;
        bool      _primed        = false;   // false until the first update() adopts the real
                                            // contact state (kills a phantom boot event)
        bool      _synced        = false;   // gates assist, see synced()

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // Close out a stride: fold the measured time into the EMA, restart the phase clock.
        void _closeStride();

        // Tuning lives in Config.h → Config::Gait.
};
