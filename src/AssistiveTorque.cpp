#include "Config.h"
#include "AssistiveTorque.h"

// Constructor
// --------------------------------------------------------------------------------------------
AssistiveTorque::AssistiveTorque(String name)
    : _name(name) {}

// Helper Functions
// --------------------------------------------------------------------------------------------
// One smooth raised-cosine bump: 1.0 at `center`, tapering to 0 at ±`halfwidth`, 0 elsewhere.
// Distance is measured circularly on the [0, 1) stride so a bump near phase 0/1 wraps cleanly.
float AssistiveTorque::raisedCosBump(float phase, float center, float halfwidth) {
    float d = phase - center;
    d -= roundf(d);                       // wrap to [-0.5, 0.5] — circular distance on the stride
    if (fabsf(d) >= halfwidth) return 0.0f;
    return 0.5f * (1.0f + cosf(PI * d / halfwidth));
}

// Public Methods
// --------------------------------------------------------------------------------------------
float AssistiveTorque::compute(float phase, float gain) const {
    float flex = Config::Assist::FLEX_PEAK_NM * raisedCosBump(phase, Config::Assist::FLEX_CENTER, Config::Assist::FLEX_HALFWIDTH);
    float ext  = Config::Assist::EXT_PEAK_NM  * raisedCosBump(phase, Config::Assist::EXT_CENTER,  Config::Assist::EXT_HALFWIDTH);
    return gain * (flex - ext);           // + = flexion assist, − = extension assist
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void AssistiveTorque::printData(float phase, float gain) const {
    Serial.print(_name);
    Serial.print(" | phase: ");  Serial.print(phase, 3);
    Serial.print(" | tau: ");    Serial.print(compute(phase, gain), 3);
    Serial.println(" Nm");
}
