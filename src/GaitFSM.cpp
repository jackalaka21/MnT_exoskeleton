#include <Arduino.h>

#include "Config.h"
#include "GaitFSM.h"

// Constructor
// --------------------------------------------------------------------------------------------
GaitFSM::GaitFSM(String name, float dt)
    : _name(name)
    , _dt(dt)
    , _stride_period(Config::Gait::NOMINAL_STRIDE_S)
{}

// Helper Functions
// --------------------------------------------------------------------------------------------
void GaitFSM::_closeStride() {
    // Only fold physiologically plausible strides into the estimate; a bounced FSR or a
    // stumble produces an implausibly short/long time that would corrupt the EMA.
    if (_t_in_stride >= Config::Gait::MIN_STRIDE_S && _t_in_stride <= Config::Gait::MAX_STRIDE_S) {
        _stride_period = Config::Gait::STRIDE_EMA_ALPHA * _t_in_stride
                       + (1.0f - Config::Gait::STRIDE_EMA_ALPHA) * _stride_period;
    }
    _t_in_stride = 0.0f;
    _phase       = 0.0f;
}

// Public Methods
// --------------------------------------------------------------------------------------------
void GaitFSM::update(bool heel_contact, bool toe_contact) {
    // Clear one-shot events; they are re-raised below only on the cycle they occur.
    _heel_strike = false;
    _toe_off     = false;

    switch (_state) {

        case GaitState::STANCE:
            // Heel lifts while the toe stays loaded → push-off. FSR-only: no hip-velocity gate.
            if (!heel_contact && toe_contact) {
                _state = GaitState::PRE_SWING;
            }
            break;

        case GaitState::PRE_SWING:
            // Both sensors clear → foot has left the ground, swing begins.
            if (!heel_contact && !toe_contact) {
                _state   = GaitState::SWING;
                _toe_off = true;
                break;
            }
            // Heel came back down (aborted step / weight shift) → back to stance.
            if (heel_contact) {
                _state = GaitState::STANCE;
            }
            break;

        case GaitState::SWING:
            // Heel strike ends the stride: this is the phase-0 reference event.
            if (heel_contact) {
                _state       = GaitState::STANCE;
                _heel_strike = true;
                _closeStride();   // measures the stride just completed, resets the phase clock
            }
            break;
    }

    // Advance the phase clock every cycle and normalise against the current stride estimate.
    // Held at 1.0 if the stride overruns the estimate (e.g. a pause mid-step) rather than
    // wrapping, so the controller sees "end of stride" instead of a false restart.
    _t_in_stride += _dt;
    _phase = _t_in_stride / _stride_period;
    if (_phase > 1.0f) _phase = 1.0f;
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void GaitFSM::printData() {
    static const char* const STATE_NAMES[] = { "STANCE", "PRE_SWING", "SWING" };

    Serial.print(_name);
    Serial.print(" | state: ");   Serial.print(STATE_NAMES[static_cast<uint8_t>(_state)]);
    Serial.print(" | phase: ");   Serial.print(_phase, 3);
    Serial.print(" | stride: ");  Serial.print(_stride_period, 2);
    Serial.print(" s | HS: ");    Serial.print(_heel_strike ? "1" : "0");
    Serial.print(" | TO: ");      Serial.println(_toe_off ? "1" : "0");
}
