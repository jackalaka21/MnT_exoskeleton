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
        constexpr uint8_t THRESHOLD  = 50;   // 0–255
        constexpr uint8_t HYSTERESIS = 10;   // 0–255
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
    }

    // ------------------------------------------------------------------------
    //  Assistive torque profile — Catmull-Rom spline over the stride [0,1]
    // ------------------------------------------------------------------------
    // Control points hold the BIOLOGICAL level-ground hip-torque ground truth
    // (Camargo et al. "MODE" dataset, red Ground-Truth trace), in Nm per kg body
    // mass. A periodic Catmull-Rom spline is drawn smoothly THROUGH every point;
    // it is converted to a joint-torque command in AssistiveTorque::compute() by
    // NMKG_TO_NM (= body mass × assist ratio, with the dataset's + = extension
    // sign flipped to our + = flexion). Reshape the assist by editing this table.
    //   • phase         : fraction of the stride, 0 = heel strike. MUST be sorted
    //                     ascending and lie in [0, 1). Include one at phase 0.0.
    //   • torque_per_kg : biological hip torque at that instant, Nm/kg (dataset sign).
    // The curve is periodic: it wraps from the last point back to the first, so
    // the profile is continuous across the heel-strike seam.
    namespace Assist {
        struct TorquePoint { float phase; float torque_per_kg; };

        constexpr TorquePoint TORQUE_POINTS[] = {
            { 0.00f,  0.50f },   // heel strike
            { 0.05f,  0.57f },   // early-stance extensor peak
            { 0.10f,  0.52f },
            { 0.15f,  0.42f },
            { 0.20f,  0.30f },
            { 0.25f,  0.18f },
            { 0.30f,  0.05f },   // ~zero crossing near 33%
            { 0.35f, -0.15f },
            { 0.40f, -0.48f },
            { 0.45f, -0.78f },
            { 0.48f, -0.90f },   // pre-swing flexor trough (deepest)
            { 0.55f, -0.72f },
            { 0.60f, -0.42f },
            { 0.65f, -0.22f },
            { 0.70f, -0.05f },   // ~zero crossing near 71%
            { 0.75f,  0.10f },
            { 0.80f,  0.25f },
            { 0.85f,  0.35f },
            { 0.90f,  0.42f },
            { 0.95f,  0.47f },   // rises back toward +0.50 at the seam
        };
        constexpr int TORQUE_N = sizeof(TORQUE_POINTS) / sizeof(TORQUE_POINTS[0]);

        // Per-kg biological profile → joint-torque command (Nm).
        //   tau = gain × NMKG_TO_NM × spline(phase)
        // NMKG_TO_NM folds in the wearer's mass, the assist ratio (fraction of the
        // biological moment the exo delivers), and the + = extension → + = flexion
        // sign flip. At 0.90 Nm/kg peak this gives ≈ 0.90 × 80 × 0.07 ≈ 5.0 Nm.
        constexpr float BODY_MASS_KG = 80.0f;   // wearer mass used to scale the per-kg profile
        constexpr float ASSIST_RATIO = 0.07f;   // exo delivers this fraction of the biological moment
        constexpr float NMKG_TO_NM   = -ASSIST_RATIO * BODY_MASS_KG;   // incl. sign flip

        // Master scale + safety clamp applied in the Controller.
        constexpr float GAIN       = 1.0f;   // 0..1 master scale — lower for a gentler first run
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
        constexpr bool  FALL_DETECT_ENABLED = false;   // TEMP: off while testing the spline only

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
    }

}
