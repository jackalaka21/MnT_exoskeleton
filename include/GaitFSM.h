#pragma once

#include <Arduino.h>

#include "Config.h"

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

        // Estimated progress through the CURRENT swing, from cadence: time since toe-off divided by
        // the expected swing duration (SWING_FRACTION × measured stride). ~0 just after toe-off,
        // ~0.5–0.6 near peak-flexion reversal, →1.0 at the estimated heel strike. Only meaningful
        // while state() == SWING; the FSRs are blind mid-air, so this timer is the only swing signal.
        float swingProgress() const {
            float expected = Config::Gait::SWING_FRACTION * _stride_period;
            return (expected > 1e-3f) ? (_t_in_swing / expected) : 0.0f;
        }

        // Measured stride period in seconds (EMA-smoothed). Falls back to NOMINAL_STRIDE_S
        // until the first full stride has been observed.
        float stridePeriod() const { return _stride_period; }

        // Edge events — true for exactly one cycle after they occur.
        bool heelStrike() const { return _heel_strike; }   // SWING → stance (initial contact)
        bool toeOff()     const { return _toe_off; }        // stance → SWING (foot airborne)

        // Gait sync — true only while the state sequence looks like a real stride. It is SET by a
        // plausible heel-first initial contact (SWING → LOADING / MID_STANCE) and CLEARED by a
        // sensor-fault signature: reaching TERMINAL_STANCE (which drives the largest push-off
        // torque) any way other than through MID_STANCE — e.g. a toe-only contact appearing from
        // mid-air (false toe FSR). SAFETY: the controller must withhold assist while this is false,
        // so a faulty FSR cannot command an unexpected torque. Starts false (no assist until a real
        // heel strike is seen), and re-arms on the next clean heel strike.
        bool synced() const { return _synced; }

        // Activity watchdog. True only while gait edge events keep arriving; goes false once no
        // heel strike / toe off has occurred for IDLE_TIMEOUT_STRIDES × the measured stride period
        // — i.e. the wearer has stopped walking, the foot is off the ground, or the FSRs never fire
        // (bench). Scaling by the stride keeps the cutoff quick at normal cadence without tripping a
        // slow walker. SAFETY-CRITICAL: the controller must command ZERO assist whenever this is
        // false, else the free-running phase clock keeps issuing torque and spins an unloaded joint.
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

        GaitState _state         = GaitState::MID_STANCE;   // both FSRs loaded — safe standing default
        float     _phase         = 0.0f;    // [0, 1] over the stride
        float     _t_in_stride   = 0.0f;    // seconds elapsed since the last heel strike
        float     _t_in_swing    = 0.0f;    // seconds elapsed since the last toe-off (swing timer)
        float     _t_since_event = 1e6f;    // seconds since the last edge event (starts "idle")
        float     _stride_period = 0.0f;    // EMA-smoothed stride time, seconds

        bool      _heel_strike   = false;   // one-cycle event flags
        bool      _toe_off       = false;
        bool      _primed        = false;   // false until the first update() adopts the real
                                            // contact state (suppresses a phantom boot edge event)
        bool      _synced        = false;   // false until a plausible heel strike; gates assist

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // Register a completed stride: update the smoothed period and restart the phase clock.
        void _closeStride();

        // Tuning constants live in Config.h → Config::Gait (stride window, EMA).
};
