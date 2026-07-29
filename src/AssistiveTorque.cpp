#include "Config.h"
#include "AssistiveTorque.h"

// Constructor
// --------------------------------------------------------------------------------------------
AssistiveTorque::AssistiveTorque(String name)
    : _name(name) {}

// Public Methods
// --------------------------------------------------------------------------------------------
// Everything is a fraction of the peak, so scaling PEAK_TORQUE_NM scales the whole profile.
// SWING's table entry is 0 — the leg swings free, see the note in Config::Assist.
float AssistiveTorque::compute(GaitState state, float gain) const {
    const float peak = gain * Config::Assist::PEAK_TORQUE_NM;
    return peak * Config::Assist::STATE_TORQUE_FRAC[static_cast<uint8_t>(state)];
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void AssistiveTorque::printData(GaitState state, float gain) const {
    Serial.print(_name);
    Serial.print(" | state: ");  Serial.print(static_cast<uint8_t>(state));
    Serial.print(" | tau: ");    Serial.print(compute(state, gain), 3);
    Serial.println(" Nm");
}
