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

    // First call: adopt the real contact state without emitting an edge event. The FSM boots in
    // MID_STANCE (safe standing default), but the feet may actually read anything at power-up
    // (e.g. no contact on a bench = SWING). Without this, that boot mismatch fires a phantom
    // toe-off / heel-strike, which would trip the walking() watchdog and spin the motor at startup.
    if (!_primed) {
        _primed = true;
        _state  = next;
        return;   // stay idle: no events, walking() false, zero assist until real gait begins
    }

    // Edge events + gait-sync validation (see synced()).
    //   • initial ground contact after swing → HEEL STRIKE: phase-0 reference; close the stride.
    //   • foot leaving the ground            → TOE OFF: the foot has gone airborne.
    if (_state == GaitState::SWING && next != GaitState::SWING) {
        // Entering stance from swing. A plausible initial contact is heel-first (LOADING) or a
        // fast heel+toe (MID_STANCE) — both have the heel loaded. A toe-ONLY contact from mid-air
        // (→ TERMINAL_STANCE) is the false-toe signature and is caught by the sequence guard below.
        if (next == GaitState::LOADING || next == GaitState::MID_STANCE) {
            _heel_strike = true;
            _synced      = true;
            _closeStride();   // measures the stride just completed, resets the phase clock
        }
    }
    else if (_state != GaitState::SWING && next == GaitState::SWING) {
        _toe_off = true;
    }

    // Sequence guard: TERMINAL_STANCE drives the largest (flexion) torque, so guard how it is
    // reached — but do NOT require the foot to pass through every prior state. A quick heel-to-toe
    // roll can skip MID_STANCE entirely (foot-flat never registers on both FSRs at once), so
    // LOADING → TERMINAL_STANCE is a legal stride and must keep sync. The ONLY fault signature is
    // TERMINAL_STANCE appearing straight out of SWING — a toe-only contact from mid-air (false toe
    // FSR) with no preceding heel load. That still drops sync, so the controller withholds assist
    // until a clean heel strike re-syncs the gait.
    if (next == GaitState::TERMINAL_STANCE && _state == GaitState::SWING) {
        _synced = false;
    }

    _state = next;

    // Activity watchdog. Reset on every edge event; otherwise it grows unbounded. Once it passes
    // IDLE_TIMEOUT_S, walking() reads false and the controller cuts assist — this is what stops
    // the phase clock from driving torque when no FSR ever fires (bench / foot in the air).
    if (_heel_strike || _toe_off) _t_since_event = 0.0f;
    else                          _t_since_event += _dt;

    // Swing timer for the cadence-based push-down. Restart it at toe-off (swing begins); it then
    // measures how far into the swing we are. Only read via swingProgress() while state == SWING.
    if (_toe_off) _t_in_swing = 0.0f;
    _t_in_swing += _dt;

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
