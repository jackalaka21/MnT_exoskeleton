#pragma once

#include <Arduino.h>

#include "MotorCAN.h"
#include "GaitFSM.h"
#include "AssistiveTorque.h"

// Assistive gait controller for both legs. Called from the two real-time tasks:
//   updateGaitPhase()      — sensor cycle: advance each leg's gait estimate from its FSRs.
//   applyAssistiveTorque() — control cycle: turn that into a torque command per drive.
//
// It reads and writes the shared VolatileData globals, so the tasks in main.cpp don't need to
// know anything about the control maths.
class Controller {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // can : shared CAN bus both drives sit on
        Controller(MotorCAN& can);

        // Public Methods
        // ----------------------------------------------------------------------------------------
        void updateGaitPhase();          // sensor cycle
        void applyAssistiveTorque();     // control cycle (only commands while the drives are armed)

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        MotorCAN&       _can;
        GaitFSM         _gait_left;    // dt must match the sensor task rate
        GaitFSM         _gait_right;
        AssistiveTorque _assist;       // gait state → torque, shared by both legs

        // Standing detector: how long both feet have been loaded, and the resulting flag. While
        // standing, both legs get zero assist. Set in updateGaitPhase(), used in _controlLeg().
        float _double_support_s = 0.0f;
        bool  _standing         = true;   // boots standing → no assist

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // One leg: read feedback, work out the assist torque, clamp, send. Publishes the leg's
        // angle / velocity / torque to the shared globals.
        void _controlLeg(uint8_t node, GaitFSM& gait, float dir,
                         volatile float* angle, volatile float* velocity, volatile float* tau_cmd);

        // Tuning lives in Config.h → Config::Assist.
};
