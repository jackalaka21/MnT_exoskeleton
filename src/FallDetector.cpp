#include "Config.h"
#include "FallDetector.h"

// Constructor
// --------------------------------------------------------------------------------------------
FallDetector::FallDetector(float dt)
    : _dt(dt) {}

// Public Methods
// --------------------------------------------------------------------------------------------
bool FallDetector::update(float pelvis_pitch_deg) {
    // Once latched, stay latched — a fall is not something we quietly recover from.
    if (_fallen) return true;

    if (!Config::Safety::FALL_DETECT_ENABLED) return false;

    // "Sudden" test: how fast is the filtered angle changing this cycle (degrees per second)?
    // Skip the very first sample, when there is no previous angle to compare against yet.
    bool changing_fast = false;
    if (_have_prev) {
        float rate_dps = fabsf(pelvis_pitch_deg - _prev_angle) / _dt;
        changing_fast = (rate_dps >= Config::Safety::FALL_RATE_DPS);
    }
    _prev_angle = pelvis_pitch_deg;
    _have_prev  = true;

    // "Fallen" backstop: is the trunk tilted far past upright?
    bool tilted_over = Config::Safety::FALL_TILT_ENABLED
                    && fabsf(pelvis_pitch_deg) >= Config::Safety::FALL_TILT_DEG;

    // Require the condition to hold for a few cycles in a row so a single noisy sample can't
    // trip the exo. A real fall easily sustains it.
    if (changing_fast || tilted_over) {
        _confirm++;
        if (_confirm >= Config::Safety::FALL_CONFIRM_SAMPLES) _fallen = true;
    } else {
        _confirm = 0;
    }

    return _fallen;
}

void FallDetector::reset() {
    _prev_angle = 0.0f;
    _have_prev  = false;
    _confirm    = 0;
    _fallen     = false;
}
