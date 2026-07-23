#pragma once

#include <Arduino.h>

// Fall detector — watches the fused pelvis pitch (the Madgwick-filtered IMU angle) for a fall.
//
// A fall is flagged when the pitch either changes too FAST (a sudden topple) or ends up too FAR
// from upright, and that condition holds for a few sensor cycles in a row (debounce). Once a fall
// is confirmed the detector LATCHES: fallen() stays true until reset(), so the safety supervisor
// can drop the exo into its safe state and keep it there.
//
// update() is called once per sensor cycle with the latest fused pelvis pitch in degrees.
// Thresholds live in Config.h → Config::Safety.
class FallDetector {
    public:
        // Constructor
        // ----------------------------------------------------------------------------------------
        // dt : sensor sample period in seconds (used to turn the angle change into a rate).
        FallDetector(float dt);

        // Public Methods
        // ----------------------------------------------------------------------------------------
        // Feed the latest fused pelvis pitch (degrees). Returns true once a fall is confirmed.
        bool update(float pelvis_pitch_deg);

        bool fallen() const { return _fallen; }

        // Clear a latched fall and the running state (e.g. after the wearer is helped back up).
        void reset();

    private:
        // Internal Variables
        // ----------------------------------------------------------------------------------------
        float   _dt;
        float   _prev_angle = 0.0f;   // last pitch, for the change-per-cycle rate
        bool    _have_prev  = false;  // false until the first sample seeds _prev_angle
        uint8_t _confirm    = 0;      // consecutive cycles the fall condition has held
        bool    _fallen     = false;  // latched once a fall is confirmed
};
