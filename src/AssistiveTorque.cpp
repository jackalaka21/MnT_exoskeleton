#include "Config.h"
#include "AssistiveTorque.h"

// Constructor
// --------------------------------------------------------------------------------------------
AssistiveTorque::AssistiveTorque(String name)
    : _name(name) {}

// Helper Functions
// --------------------------------------------------------------------------------------------
// Keep an index inside the control-point array. The profile is a loop, so going past the last
// point comes back to the first, and going before the first wraps to the last.
static int wrapIndex(int i, int count) {
    while (i < 0)      i += count;
    while (i >= count) i -= count;
    return i;
}

// Smooth curve that passes through every control point (a Catmull-Rom / cubic spline).
// Returns the per-kg biological torque (Nm/kg) at the given stride phase.
float AssistiveTorque::splineTorque(float phase) {
    const Config::Assist::TorquePoint* points = Config::Assist::TORQUE_POINTS;
    const int count = Config::Assist::TORQUE_N;

    // Keep phase inside one stride [0, 1).
    while (phase >= 1.0f) phase -= 1.0f;
    while (phase <  0.0f) phase += 1.0f;

    // Find which segment the phase lands in: the first point whose phase is still ahead of us.
    // If none is (phase is past the last point), we are in the final segment that loops back
    // over the start of the next stride.
    int seg = count - 1;
    for (int k = 0; k < count - 1; k++) {
        if (phase < points[k + 1].phase) {
            seg = k;
            break;
        }
    }

    // Phase of the two points bounding this segment. The looping segment ends one whole stride
    // after the first point (e.g. 0.95 -> 1.00).
    float startPhase = points[seg].phase;
    float endPhase;
    if (seg == count - 1) endPhase = points[0].phase + 1.0f;
    else                  endPhase = points[seg + 1].phase;

    // How far we are across this segment, from 0 at the start point to 1 at the end point.
    float t = (phase - startPhase) / (endPhase - startPhase);

    // The four points the spline needs: one before the segment, the two ends, and one after.
    float prev  = points[wrapIndex(seg - 1, count)].torque_per_kg;
    float curr  = points[seg].torque_per_kg;
    float next  = points[wrapIndex(seg + 1, count)].torque_per_kg;
    float after = points[wrapIndex(seg + 2, count)].torque_per_kg;

    // Catmull-Rom rule: the slope at a point is half the gap between its two neighbours. This is
    // what makes the curve pass through the points smoothly instead of with sharp corners.
    float slopeStart = 0.5f * (next  - prev);
    float slopeEnd   = 0.5f * (after - curr);

    // Blend from curr to next across the segment using those two slopes (cubic Hermite).
    float t2 = t * t;
    float t3 = t2 * t;
    return (2.0f * t3 - 3.0f * t2 + 1.0f) * curr
         + (t3 - 2.0f * t2 + t)           * slopeStart
         + (-2.0f * t3 + 3.0f * t2)       * next
         + (t3 - t2)                      * slopeEnd;
}

// Public Methods
// --------------------------------------------------------------------------------------------
float AssistiveTorque::compute(float phase, float gain) const {
    // The table holds the biological torque per kg. Turn it into a motor command by scaling for
    // the wearer's weight and assist level, and flipping the sign to our "+ = flexion" convention.
    // All of that is packed into NMKG_TO_NM.
    return gain * Config::Assist::NMKG_TO_NM * splineTorque(phase);
}

// Serial Monitor (for debugging only)
// --------------------------------------------------------------------------------------------
void AssistiveTorque::printData(float phase, float gain) const {
    Serial.print(_name);
    Serial.print(" | phase: ");  Serial.print(phase, 3);
    Serial.print(" | tau: ");    Serial.print(compute(phase, gain), 3);
    Serial.println(" Nm");
}
