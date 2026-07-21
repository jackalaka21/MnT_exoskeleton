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
    //  Assistive torque profile — OpenExo-style raised-cosine bumps over [0,1]
    // ------------------------------------------------------------------------
    // Centres/half-widths are fractions of the stride (0 = heel strike);
    // magnitudes in Nm. CENTER = percent-gait shift, HALFWIDTH = rise time.
    // The bumps don't overlap, so peak |torque| = max(EXT_PEAK_NM, FLEX_PEAK_NM).
    // Sign convention: + = flexion assist, − = extension assist.
    namespace Assist {
        constexpr float EXT_PEAK_NM    = 4.0f;
        constexpr float EXT_CENTER     = 0.30f;   // ~mid-stance
        constexpr float EXT_HALFWIDTH  = 0.22f;
        constexpr float FLEX_PEAK_NM   = 5.0f;
        constexpr float FLEX_CENTER    = 0.65f;   // ~toe-off / early swing
        constexpr float FLEX_HALFWIDTH = 0.22f;

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
