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
    // Only believe plausible strides — a bounced FSR or a stumble would poison the EMA.
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
    // One-shot events — re-raised below only on the cycle they happen.
    _heel_strike = false;
    _toe_off     = false;

    // Contact combination maps straight onto the gait state, no history needed.
    GaitState next;
    if      ( heel_contact && !toe_contact) next = GaitState::LOADING;          // (1,0)
    else if ( heel_contact &&  toe_contact) next = GaitState::MID_STANCE;       // (1,1)
    else if (!heel_contact &&  toe_contact) next = GaitState::TERMINAL_STANCE;  // (0,1)
    else                                    next = GaitState::SWING;            // (0,0)

    // First call: take the real contact state without firing an event. We boot in MID_STANCE but
    // the feet could read anything at power-up (no contact on a bench = SWING), and that mismatch
    // would otherwise fake a toe-off / heel strike and spin the motor at startup.
    if (!_primed) {
        _primed = true;
        _state  = next;
        return;   // stay idle until real gait starts
    }

    // Edge events: entering stance from swing = heel strike (phase 0), leaving the ground = toe off.
    if (_state == GaitState::SWING && next != GaitState::SWING) {
        // A real initial contact is heel-first (LOADING) or a fast heel+toe (MID_STANCE) — both
        // have the heel loaded. Toe-only from mid-air is the fault case, caught just below.
        if (next == GaitState::LOADING || next == GaitState::MID_STANCE) {
            _heel_strike = true;
            _synced      = true;
            _closeStride();
        }
    }
    else if (_state != GaitState::SWING && next == GaitState::SWING) {
        _toe_off = true;
    }

    // Sequence guard. TERMINAL_STANCE drives the biggest torque, so watch how we got there.
    // Reaching it straight from SWING means a toe FSR fired mid-air with no heel load first —
    // drop sync so the controller withholds assist until a clean heel strike.
    if (next == GaitState::TERMINAL_STANCE && _state == GaitState::SWING) {
        _synced = false;
    }

    _state = next;

    // Idle watchdog. Reset on every edge event, otherwise it grows until walking() reads false.
    // This is what stops the phase clock driving torque when no FSR ever fires.
    if (_heel_strike || _toe_off) _t_since_event = 0.0f;
    else                          _t_since_event += _dt;

    // Swing timer. Restarts at toe-off; read via swingProgress(). Telemetry only now that the
    // swing push-down is gone — nothing in the torque path uses it.
    if (_toe_off) _t_in_swing = 0.0f;
    _t_in_swing += _dt;

    // Phase clock, normalised against the current stride estimate. Held at 1.0 on overrun (e.g. a
    // pause mid-step) instead of wrapping, so the controller sees "end of stride", not a restart.
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
