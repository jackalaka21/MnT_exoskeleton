#pragma once

#include <Arduino.h>

#include "GaitFSM.h"   // GaitState

// Assistive hip torque — CONTACT-DRIVEN.
//
// The target torque is a direct lookup on the current gait state (Config::Assist::STATE_TORQUE_NM),
// which is itself a decode of the foot's heel/toe FSRs. Sign convention: + = flexion assist,
// − = extension assist. There is no stride phase and no timer here — the torque is a pure function
// of what the foot is doing right now, which makes it predictable and bench-verifiable. The
// Controller is responsible for slew-limiting toward this target, gating on GaitFSM::walking(),
// and clamping to the safety backstop.
class AssistiveTorque {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // name : label prefix used in Serial output (e.g. "left", "right")
        AssistiveTorque(String name = "assist");

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Target joint torque (Nm) for the given gait state. In SWING the FSRs are blind, so
        // `swing_progress` (from GaitFSM::swingProgress()) times a cadence-based push-down; it is
        // ignored in every other state. `gain` is a 0..1 master scale for safe ramp-up.
        float compute(GaitState state, float swing_progress, float gain = 1.0f) const;

        // Serial Monitor (for debugging only)
        // ----------------------------------------------------------------------------------------
        void printData(GaitState state, float swing_progress, float gain = 1.0f) const;

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        String _name;

        // The per-state torque table lives in Config.h → Config::Assist::STATE_TORQUE_NM.
};
