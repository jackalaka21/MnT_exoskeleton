#pragma once

#include <Arduino.h>

#include "GaitFSM.h"   // GaitState

// Assist torque lookup. The target is a straight table read on the current gait state, which is
// itself a decode of the heel/toe FSRs — no phase, no timers, so it's easy to bench-test.
// Sign: + = flexion, − = extension. The Controller does the slew limiting, gating and clamping.
class AssistiveTorque {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // name : label for Serial output
        AssistiveTorque(String name = "assist");

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Target joint torque (Nm) for a gait state. SWING commands zero — the leg swings free.
        // `gain` is a 0..1 master scale for safe ramp-up.
        float compute(GaitState state, float gain = 1.0f) const;

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printData(GaitState state, float gain = 1.0f) const;

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String _name;

        // The torque table lives in Config.h → Config::Assist::STATE_TORQUE_FRAC.
};
