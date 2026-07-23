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

    // Decode the foot-contact combination directly into the gait phase. The two FSRs give four
    // (heel, toe) combinations, one per canonical phase — no path/history dependence needed.
    GaitState next;
    if      ( heel_contact && !toe_contact) next = GaitState::LOADING;          // (1,0)
    else if ( heel_contact &&  toe_contact) next = GaitState::MID_STANCE;       // (1,1)
    else if (!heel_contact &&  toe_contact) next = GaitState::TERMINAL_STANCE;  // (0,1)
    else                                    next = GaitState::SWING;            // (0,0)

    // Edge events are defined by entering/leaving SWING (the airborne phase):
    //   • initial ground contact after swing → HEEL STRIKE: the phase-0 reference; close the stride.
    //   • foot leaving the ground            → TOE OFF: the foot has gone airborne.
    if (_state == GaitState::SWING && next != GaitState::SWING) {
        _heel_strike = true;
        _closeStride();   // measures the stride just completed, resets the phase clock
    }
    else if (_state != GaitState::SWING && next == GaitState::SWING) {
        _toe_off = true;
    }

    _state = next;

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
    static const char* const STATE_NAMES[] = { "LOADING", "MID_STANCE", "TERMINAL_STANCE", "SWING" };

    Serial.print(_name);
    Serial.print(" | state: ");   Serial.print(STATE_NAMES[static_cast<uint8_t>(_state)]);
    Serial.print(" | phase: ");   Serial.print(_phase, 3);
    Serial.print(" | stride: ");  Serial.print(_stride_period, 2);
    Serial.print(" s | HS: ");    Serial.print(_heel_strike ? "1" : "0");
    Serial.print(" | TO: ");      Serial.println(_toe_off ? "1" : "0");
}
