#pragma once

#include <Arduino.h>

// ============================================================================
//  Config.h — CENTRAL TUNING MENU
// ============================================================================
// One place to tune the whole exoskeleton. Every value here is a compile-time
// constant: edit a number, re-flash, and the change propagates to whichever
// module reads it. Grouped by subsystem so related knobs sit together.
//
// This file is for VALUES YOU TUNE (gains, thresholds, filter strengths, torque
// shape). It is deliberately separate from:
//   • VolatileData.h — live runtime state shared between tasks (volatile).
//   • Pin.h          — hardware wiring: pin numbers, I2C addresses, CAN node IDs.
//
// Nothing here is `volatile`: these are read once at compile time, not changed
// while the firmware runs.
// ============================================================================

namespace Config {

    // ------------------------------------------------------------------------
    //  Loop rates
    // ------------------------------------------------------------------------
    // Sensor + control loops run at this rate. Used to derive the filter sample
    // periods below. NOTE: the FreeRTOS task periods in main.cpp (pdMS_TO_TICKS)
    // must be kept in sync with these by hand — changing a number here does not
    // move the task tick.
    namespace Rates {
        constexpr float SENSOR_HZ  = 100.0f;              // sensorTask + Madgwick + GaitFSM
        constexpr float CONTROL_HZ = 100.0f;              // controlTask
        constexpr float SENSOR_DT  = 1.0f / SENSOR_HZ;    // seconds — GaitFSM / Madgwick dt
    }

    // ------------------------------------------------------------------------
    //  FSR — foot force sensors
    // ------------------------------------------------------------------------
    namespace FSR {
        // EMA spike filter. Lower = smoother (rejects spikes harder) but more
        // lag on real foot strikes; higher = snappier, less filtering.
        constexpr float   EMA_ALPHA = 0.2f;
        // Contact detection: contact asserts above THRESHOLD+HYSTERESIS and
        // clears below THRESHOLD-HYSTERESIS (deadband stops chatter at the edge).
        // Per-sensor threshold — LOWER = MORE SENSITIVE (triggers with less force).
        // The forefoot (toe) sits lower so it picks up the lighter push-off load.
        // Keep any threshold above HYSTERESIS so THRESHOLD−HYSTERESIS can't underflow.
        constexpr uint8_t THRESHOLD     = 90;   // 0–255 — default / heel
        constexpr uint8_t TOE_THRESHOLD = 25;   // 0–255 — forefoot (more sensitive)
        constexpr uint8_t HYSTERESIS    = 10;   // 0–255

        // Per-SENSOR sensitivity gain. The reading is multiplied by this before the
        // threshold compare, so it rescales a physically less/more sensitive sensor
        // onto the common thresholds above (result is clamped to 255). >1.0 makes a
        // sluggish sensor trigger with less force; 1.0 = no change. Each of the four
        // FSRs is tuned independently — press each one with a known load and raise its
        // gain until it asserts contact at a force comparable to the others.
        constexpr float DEFAULT_GAIN     = 1.0f;   // no scaling (constructor fallback)
        constexpr float LEFT_HEEL_GAIN   = 2.0f;
        constexpr float LEFT_TOE_GAIN    = 0.5f;
        constexpr float RIGHT_HEEL_GAIN  = 1.0f;
        constexpr float RIGHT_TOE_GAIN   = 1.0f;

        // Debounce: a new contact state must persist this many consecutive samples before it is
        // adopted, so a single-cycle spike (false trigger) or dropout (missed trigger) can't flip
        // the contact flag. At 100 Hz, 3 samples = 30 ms — negligible gait lag, big noise rejection.
        constexpr uint8_t DEBOUNCE_SAMPLES = 3;
    }

    // ------------------------------------------------------------------------
    //  Gait FSM — stride phase estimator (per leg)
    // ------------------------------------------------------------------------
    namespace Gait {
        // Stride period used before the first stride is measured, and the
        // plausibility window a measured stride must fall inside to be accepted
        // into the EMA (rejects FSR chatter / stumbles).
        // ~0.5 s ≈ very fast jog, ~2.5 s ≈ slow shuffle.
        constexpr float NOMINAL_STRIDE_S = 1.10f;
        constexpr float MIN_STRIDE_S     = 0.50f;
        constexpr float MAX_STRIDE_S     = 2.50f;

        // EMA weight for the stride-period estimate: higher = adapts faster but noisier.
        constexpr float STRIDE_EMA_ALPHA = 0.30f;

        // Swing occupies roughly this fraction of the stride (toe-off → next heel strike). Used to
        // turn "time since toe-off" into a swing-progress estimate for the cadence-timed push-down,
        // since the FSRs give no information while the foot is airborne. ~0.40 is typical walking.
        constexpr float SWING_FRACTION = 0.40f;

        // Activity watchdog: if no gait edge event (heel strike / toe off) arrives for this many
        // stride periods, GaitFSM::walking() goes false and the controller commands ZERO assist.
        // Scaling by the MEASURED stride (instead of a fixed timeout) keeps the cutoff snappy at
        // normal cadence yet still tolerant of a slow shuffle. The longest legitimate gap between
        // edge events is ~0.6 stride (the stance interval), so this must stay above ~0.6 — 1.25
        // leaves margin while cutting assist within ~1 stride of the wearer stopping. Lower it for
        // a faster cutoff, but not below ~0.7 or slow walking will false-trip mid-stance.
        constexpr float IDLE_TIMEOUT_STRIDES = 1.25f;

        // Standing detector (whole-body, FSR-only). While WALKING, both feet are loaded at the same
        // time only briefly (double-support, ~0.1–0.2 s per stride); while STANDING, both feet stay
        // loaded indefinitely. So if BOTH feet ("heel OR toe" on each leg) stay continuously loaded
        // for longer than this, the wearer is standing → the controller zeroes assist on BOTH legs.
        // Keep this ABOVE the longest physiological double-support (~0.2 s at slow cadence) so a real
        // stride can't trip it, but low enough to cut assist quickly once the wearer stops. This is a
        // POSITIVE stop signal, unlike the silence-based IDLE_TIMEOUT_STRIDES watchdog (kept as a
        // backstop): postural sway / FSR chatter can't fool it because it requires SUSTAINED contact.
        constexpr float STANDING_DOUBLE_SUPPORT_S = 0.45f;
    }

    // ------------------------------------------------------------------------
    //  Assistive torque — CONTACT-DRIVEN: one torque target per gait state
    // ------------------------------------------------------------------------
    // The commanded joint torque is a direct lookup on the current gait state,
    // which is itself a direct decode of the heel/toe FSRs (see GaitFSM). No
    // stride phase, no timer — the torque responds to what the foot is actually
    // doing, so it is predictable and easy to validate on a bench. The controller
    // ramps toward the target at TAU_RATE_NM_S (no torque steps) and only applies
    // it while GaitFSM::walking() is true (so standing on both feet, which reads
    // MID_STANCE, does not command a constant extension torque).
    //
    // Sign convention (final joint torque): + = FLEXION assist, − = EXTENSION assist.
    namespace Assist {
        // ── THE testing knob ─────────────────────────────────────────────────────────────────
        // Peak torque (Nm) the profile reaches at its strongest point. The whole profile below is
        // expressed as FRACTIONS of this, so raising/lowering this one number scales every state
        // (and the swing push-down) proportionally. Start low for a gentle first run and raise it
        // as you gain confidence. Keep it ≤ TAU_MAX_NM (the hard safety clamp) or the clamp bites.
        constexpr float PEAK_TORQUE_NM = 3.0f;

        // Profile SHAPE — fractions of PEAK_TORQUE_NM, one per gait state. Index by
        // static_cast<uint8_t>(GaitState); order MUST match the enum: LOADING, MID_STANCE,
        // TERMINAL_STANCE, SWING. Sign = our convention (+ = flexion, − = extension). The ±1.0
        // entry defines the peak; keep every |fraction| ≤ 1.0.
        constexpr float STATE_TORQUE_FRAC[4] = {
            -0.80f,   // LOADING          heel-only: extension assist, weight acceptance
            -0.45f,  // MID_STANCE       both:      light extension while body passes over the foot
            +1.00f,   // TERMINAL_STANCE  toe-only:  flexion assist, push-off  (this is the peak)
             0.0f,    // SWING            airborne:  base is 0 — swing is handled specially below
        };

        // SWING push-down (cadence-timed), also a fraction of PEAK_TORQUE_NM. The foot is airborne
        // so the FSRs are blind; instead we estimate swing progress from cadence
        // (GaitFSM::swingProgress()) and, only AFTER the estimated peak-flexion reversal, drive the
        // leg DOWN toward heel strike with a ramping extension torque. Pushing only in late swing
        // assists the leg's own descent rather than fighting a still-rising leg.
        //   SWING_PUSH_START : swing progress [0,1] at which the push-down begins (~reversal).
        //                      Bias it LATE for safety — earlier risks resisting the rising leg.
        //   SWING_PUSH_FRAC  : peak push-down fraction at estimated heel strike (negative = down).
        //   SWING_PUSH_END   : upper bound on swing progress. Past this the swing has lasted longer
        //                      than a plausible stride — a MISSED heel strike, a stumble, or both
        //                      FSRs dropping out while the foot is actually planted — so we WITHHOLD
        //                      the push rather than keep driving extension into a leg whose real
        //                      state we've lost. Bounds a mistimed/missed push to a short window
        //                      instead of holding until the walking() watchdog trips.
        constexpr float SWING_PUSH_START = 0.60f;
        constexpr float SWING_PUSH_FRAC  = -0.50f;
        constexpr float SWING_PUSH_END   = 1.30f;

        // Slew-rate limit on the commanded torque, Nm/s. Ramps between targets so a state change
        // never steps the current. ~40 Nm/s crosses a full-scale swing in ~0.2 s — responsive but
        // not jerky. This is an absolute rate, so it does NOT scale with PEAK_TORQUE_NM.
        constexpr float TAU_RATE_NM_S = 40.0f;

        // Extra master scale (0..1) and the hard safety clamp, applied in the Controller. GAIN
        // multiplies on top of PEAK_TORQUE_NM — for testing, prefer adjusting PEAK_TORQUE_NM and
        // leave GAIN at 1.0.
        constexpr float GAIN       = 1.0f;   // 0..1 master scale
        constexpr float TAU_MAX_NM = 6.0f;   // safety clamp on commanded joint torque, Nm
    }

    // ------------------------------------------------------------------------
    //  IMU / Madgwick orientation filter
    // ------------------------------------------------------------------------
    namespace Imu {
        // Madgwick gain — higher trusts the accelerometer more (faster settle,
        // noisier), lower trusts the gyro more (smoother, more drift).
        constexpr float MADGWICK_BETA = 0.1f;
    }

    // ------------------------------------------------------------------------
    //  Safety — fall detection from the fused pelvis pitch (filtered IMU angle)
    // ------------------------------------------------------------------------
    // A fall shows up in the pelvis pitch two ways: the angle CHANGES FAST (the
    // trunk pitches over) and/or it ends up FAR FROM UPRIGHT. FallDetector watches
    // both; when either holds for FALL_CONFIRM_SAMPLES cycles in a row it latches a
    // fall, and safetyTask puts the exo in its safe state (same as an E-stop).
    namespace Safety {
        // Trips on EITHER a sudden pelvis-pitch rate (FALL_RATE_DPS) OR tilt past FALL_TILT_DEG.
        // On a confirmed fall the exo latches into the safe state (motors disabled, control task
        // suspended) and only resumes on a power-cycle / device restart.
        constexpr bool  FALL_DETECT_ENABLED = true;

        // "Sudden" test: rate of change of the filtered pelvis pitch, degrees/sec.
        // Normal walking sways the pelvis only slowly; a topple far exceeds this.
        constexpr float FALL_RATE_DPS = 150.0f;

        // "Fallen" backstop: absolute pelvis tilt from upright, degrees. Assumes the
        // fused pitch reads ~0 when standing — disable if the IMU zero is uncertain.
        constexpr bool  FALL_TILT_ENABLED = true;
        constexpr float FALL_TILT_DEG     = 75.0f;

        // Consecutive sensor cycles the condition must hold before latching (debounce
        // against single-sample noise). At 100 Hz, 3 cycles = 30 ms.
        constexpr uint8_t FALL_CONFIRM_SAMPLES = 3;
    }

    // ------------------------------------------------------------------------
    //  Motor + gearbox — MKS XDrive Mini, joint-side ↔ motor-side conversion
    // ------------------------------------------------------------------------
    // Encoder is on the motor shaft (before the gearbox):
    //   joint angle  = motor angle  / GEAR_RATIO
    //   joint torque = current × NM_PER_AMP
    namespace Motor {
        constexpr float KT         = 0.23f;   // nameplate motor torque constant, Nm/A — reference only
        constexpr float GEAR_RATIO = 15.0f;   // gearbox reduction
        constexpr float GEAR_EFF   = 0.6f;    // gearbox efficiency estimate — reference only
        constexpr float I_MAX      = 7.0f;    // drive current limit, A (≈ 27.5 Nm at the joint)

        // Measured 2026-07-09 with the drive's current scale verified (DRV8301 gain-80 read-back OK):
        // 1.0 Nm commanded through the old estimate KT×RATIO×EFF (2.07 Nm/A) produced 1.9 Nm at the
        // joint, so the true scale is 2.07 × 1.9. Re-measure if the motor or gearbox changes.
        constexpr float NM_PER_AMP = 3.93f;

        // Per-leg torque sign, tuned empirically on the bench so each side assists (not resists)
        // in its + = flexion direction. Both currently +1. If a side ever drives the WRONG way
        // after a mechanics/wiring/firmware change, flip only that side's sign and re-verify at
        // low GAIN — do not assume the two sides always match.
        constexpr float DIR_L = +1.0f;
        constexpr float DIR_R = +1.0f;
    }

}
