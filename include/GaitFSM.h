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

        // BENCH TESTING ONLY — free-running phase clock, no sensors.
        // The phase marches 0→1 on its own at a fixed cadence (BENCH_STRIDE_S), independent of the
        // arm's angle, so the assist torque cycles and the arm swings hands-free like a gait cycle.
        // This is open-loop (it never reads the thing it moves), so unlike the angle-driven version
        // there is no feedback loop to go unstable. Swap back to update() on the real hardware.
        void updateBench();

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

        // Tuning constants
        // ----------------------------------------------------------------------------------------
        // Stride period used before the first stride is measured, and the plausibility window
        // a measured stride must fall inside to be accepted into the EMA (rejects FSR chatter
        // and stumbles). ~0.5 s ≈ very fast jog, ~2.5 s ≈ slow shuffle.
        static constexpr float NOMINAL_STRIDE_S = 1.10f;
        static constexpr float MIN_STRIDE_S     = 0.50f;
        static constexpr float MAX_STRIDE_S     = 2.50f;

        // EMA weight for the stride-period estimate: higher = adapts faster but noisier.
        static constexpr float STRIDE_EMA_ALPHA = 0.30f;

        // BENCH TESTING — cadence of the free-running phase clock (see updateBench()).
        // One full stride every BENCH_STRIDE_S seconds. Keep it slow for a safe first run.
        static constexpr float BENCH_STRIDE_S = 1.50f;
        // Phase thresholds that split the stride into the three display states.
        static constexpr float BENCH_PRESWING_PHASE = 0.60f;   // STANCE → PRE_SWING above this
        static constexpr float BENCH_SWING_PHASE    = 0.80f;   // PRE_SWING → SWING above this
};
