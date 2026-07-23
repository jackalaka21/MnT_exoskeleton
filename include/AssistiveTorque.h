#pragma once

#include <Arduino.h>

// Assistive hip torque profile — decoupled from gait estimation.
//
// The torque curve is a periodic Catmull-Rom spline drawn smoothly THROUGH a table of
// {phase, torque} control points (Config::Assist::TORQUE_POINTS): extension assist (negative)
// during stance, flexion assist (positive) around toe-off/early swing. Sign convention:
// + = flexion assist, − = extension assist.
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
        // Evaluate the periodic Catmull-Rom spline through Config::Assist::TORQUE_POINTS at the
        // given stride phase. Returns the raw (un-gained) torque in Nm.
        static float splineTorque(float phase);

        // Profile shape (the control-point table) lives in Config.h → Config::Assist.
};
