#include "Config.h"
#include "AssistiveTorque.h"

// Constructor
// --------------------------------------------------------------------------------------------
AssistiveTorque::AssistiveTorque(String name)
    : _name(name) {}

// Public Methods
// --------------------------------------------------------------------------------------------
float AssistiveTorque::compute(GaitState state, float swing_progress, float gain) const {
    // SWING is special: the foot is airborne so there is no contact to key on. Use the cadence
    // estimate to push the leg DOWN toward heel strike, but only AFTER the estimated peak-flexion
    // reversal (SWING_PUSH_START), ramping the extension torque up to its peak at heel strike.
    // All torques are a fraction of the peak; scaling PEAK_TORQUE_NM scales the whole profile.
    const float peak = gain * Config::Assist::PEAK_TORQUE_NM;

    if (state == GaitState::SWING) {
        // Push down only inside a plausible swing window. Before START we let the leg rise/reverse
        // freely; past END the swing has overrun (missed heel strike / stumble / both FSRs dropped)
        // so we withhold the push instead of driving extension into a leg whose state we've lost.
        if (swing_progress <= Config::Assist::SWING_PUSH_START) return 0.0f;
        if (swing_progress >= Config::Assist::SWING_PUSH_END)    return 0.0f;
        float ramp = (swing_progress - Config::Assist::SWING_PUSH_START)
                   / (1.0f - Config::Assist::SWING_PUSH_START);
        if (ramp > 1.0f) ramp = 1.0f;   // hold at peak through the (1.0 → END) overrun tolerance
        return peak * Config::Assist::SWING_PUSH_FRAC * ramp;
    }

    // Every other state: the per-state fraction of the peak. Sign is our convention
    // (+ = flexion, − = extension).
    return peak * Config::Assist::STATE_TORQUE_FRAC[static_cast<uint8_t>(state)];
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void AssistiveTorque::printData(GaitState state, float swing_progress, float gain) const {
    Serial.print(_name);
    Serial.print(" | state: ");  Serial.print(static_cast<uint8_t>(state));
    Serial.print(" | swing: ");  Serial.print(swing_progress, 2);
    Serial.print(" | tau: ");    Serial.print(compute(state, swing_progress, gain), 3);
    Serial.println(" Nm");
}
