#pragma once

#include <Arduino.h>

#include "MotorCAN.h"
#include "GaitFSM.h"
#include "AssistiveTorque.h"

// Assistive gait controller — the exo's control "brain" for both legs.
//
// Two responsibilities, called from the two real-time tasks:
//   • updateGaitPhase()      — turn each foot's FSR contact into a gait phase estimate.
//   • applyAssistiveTorque() — map that phase to a joint-torque command and send it to each drive.
//
// It talks to the rest of the firmware through the shared VolatileData globals: it reads the
// FSR-contact and motor-feedback globals and writes the gait, motor-state, and torque-command
// globals — so the tasks in main.cpp stay decoupled from the control maths.
class Controller {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // can : the shared motor CAN bus the controller commands both drives on.
        Controller(MotorCAN& can);

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Sensor cycle: advance each leg's gait estimate from its foot-contact flags.
        void updateGaitPhase();

        // Control cycle: drain the latest motor feedback, then command each leg's assistive
        // torque (only while the drives are armed).
        void applyAssistiveTorque();

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        MotorCAN&       _can;
        GaitFSM         _gait_left;    // dt = 0.01 s — must match the sensor task's 100 Hz rate
        GaitFSM         _gait_right;
        AssistiveTorque _assist;       // gait phase → joint torque, shared by both legs

        // Helper Functions
        // ----------------------------------------------------------------------------------------
        // One leg: read feedback, compute the assist torque for its gait phase, clamp, and send.
        // Writes the leg's angle/velocity/torque into the shared volatile outputs.
        void _controlLeg(uint8_t node, GaitFSM& gait,
                         volatile float* angle, volatile float* velocity, volatile float* tau_cmd);

        // Tuning constants (assist gain, torque clamp) live in Config.h → Config::Assist.
};
