#pragma once

#include <Arduino.h>

// Assistive hip torque profile — decoupled from gait estimation.
//
// OpenExo-style shape: two smooth (raised-cosine) bumps over the gait phase [0, 1]:
//   • EXTENSION assist bump during stance            → negative torque
//   • FLEXION  assist bump around toe-off/early swing → positive torque
// Sign convention: + = flexion assist, − = extension assist.
//
// Takes the gait phase (e.g. from GaitFSM::phase()) and returns joint torque in Nm, so the
// profile is a pure function of phase — independent of how that phase was estimated. This keeps
// the torque shaping separate from the gait state machine; both can be tuned in isolation.
class AssistiveTorque {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // name : label prefix used in Serial output (e.g. "left", "right")
        AssistiveTorque(String name = "assist");

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Joint torque (Nm) for the given gait phase [0, 1].
        // `gain` is a 0..1 master scale for safe ramp-up. Smooth by construction — no torque steps.
        float compute(float phase, float gain = 1.0f) const;

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printData(float phase, float gain = 1.0f) const;

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String _name;

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // One raised-cosine bump: 1.0 at `center`, tapering to 0 at ±`halfwidth`, 0 elsewhere.
        static float raisedCosBump(float phase, float center, float halfwidth);

        // Tuning constants
        // ----------------------------------------------------------------------------------------
        // Profile shape — centres/half-widths are fractions of the stride (0 = heel strike);
        // magnitudes in Nm. Tune to shape the assistance (OpenExo's extension/flexion magnitude,
        // percent-gait shift = CENTER, rise time = HALFWIDTH). The bumps don't overlap, so the
        // peak |torque| is max(EXT_PEAK_NM, FLEX_PEAK_NM).
        static constexpr float EXT_PEAK_NM    = 4.0f;
        static constexpr float EXT_CENTER     = 0.30f;   // ~mid-stance
        static constexpr float EXT_HALFWIDTH  = 0.22f;
        static constexpr float FLEX_PEAK_NM   = 5.0f;
        static constexpr float FLEX_CENTER    = 0.65f;   // ~toe-off / early swing
        static constexpr float FLEX_HALFWIDTH = 0.22f;
};
