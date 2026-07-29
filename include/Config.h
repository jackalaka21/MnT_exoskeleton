#pragma once

#include <Arduino.h>

// Tuning constants for the whole exo. Edit a number here, re-flash, done.
// Related headers: VolatileData.h = live shared state, Pin.h = wiring / IDs.

namespace Config {

    // Loop rates
    // --------------------------------------------------------------------------------------------
    // Task periods in main.cpp (pdMS_TO_TICKS) are separate — keep them in sync by hand.
    namespace Rates {
        constexpr float SENSOR_HZ  = 100.0f;              // sensorTask + Madgwick + GaitFSM
        constexpr float CONTROL_HZ = 100.0f;              // controlTask
        constexpr float SENSOR_DT  = 1.0f / SENSOR_HZ;    // s, used as dt by GaitFSM / Madgwick
    }

    // FSR — foot force sensors
    // --------------------------------------------------------------------------------------------
    namespace FSR {
        constexpr float EMA_ALPHA = 0.2f;   // spike filter: lower = smoother but laggier

        // Contact asserts above THRESHOLD+HYSTERESIS, clears below THRESHOLD-HYSTERESIS.
        // Lower threshold = more sensitive. Toe sits lower to catch the light push-off load.
        // Keep every threshold above HYSTERESIS so the subtraction can't underflow.
        constexpr uint8_t THRESHOLD     = 90;   // heel / default
        constexpr uint8_t TOE_THRESHOLD = 50;   // forefoot
        constexpr uint8_t HYSTERESIS    = 5;

        // Per-sensor gain, applied to the EMA-filtered reading before the threshold compare
        // (clamped to 255). Raise a sluggish sensor's gain until it trips at the same force as
        // the others. Note this scales noise with the signal — it does not filter.
        constexpr float DEFAULT_GAIN     = 1.2f;
        constexpr float LEFT_HEEL_GAIN   = 1.6f;
        constexpr float LEFT_TOE_GAIN    = 1.0f;
        constexpr float RIGHT_HEEL_GAIN  = 1.0f;
        constexpr float RIGHT_TOE_GAIN   = 1.6f;

        // A new contact state must hold this many samples before it's adopted, so one bad
        // sample can't flip the flag. 3 samples at 100 Hz = 30 ms.
        constexpr uint8_t DEBOUNCE_SAMPLES = 2;
    }

    // Gait FSM — stride phase estimator (per leg)
    // --------------------------------------------------------------------------------------------
    namespace Gait {
        // Stride period used before the first stride is measured, plus the window a measured
        // stride must land in to be believed. ~0.5 s = fast jog, ~2.5 s = slow shuffle.
        constexpr float NOMINAL_STRIDE_S = 1.10f;
        constexpr float MIN_STRIDE_S     = 0.50f;
        constexpr float MAX_STRIDE_S     = 2.50f;

        constexpr float STRIDE_EMA_ALPHA = 0.30f;   // higher = adapts faster, noisier

        // Fraction of the stride spent in swing. Turns time-since-toe-off into swing progress,
        // since the FSRs see nothing while the foot is in the air. ~0.40 is typical walking.
        constexpr float SWING_FRACTION = 0.70f;

        // Idle watchdog: no heel strike / toe off for this many stride periods → walking() goes
        // false → zero assist. Scaled by the measured stride so it's quick at normal cadence and
        // still tolerant of a shuffle. The longest legitimate gap is ~0.6 stride (stance), so
        // don't go below ~0.7 or slow walking false-trips.
        constexpr float IDLE_TIMEOUT_STRIDES = 1.25f;

        // Standing detector. Walking only loads both feet briefly (~0.1–0.2 s of double support);
        // standing loads them indefinitely. Both feet loaded longer than this → zero assist on
        // both legs. Keep above the slowest real double support so a stride can't trip it.
        constexpr float STANDING_DOUBLE_SUPPORT_S = 0.45f;
    }

    // Assistive torque — one torque target per gait state
    // --------------------------------------------------------------------------------------------
    // Torque is a straight lookup on the gait state, which is a straight decode of the heel/toe
    // FSRs. No phase, no timers — easy to predict and bench-test. The Controller ramps toward the
    // target and only applies it while walking.
    // Sign: + = flexion assist, − = extension assist.
    namespace Assist {
        // Main testing knob. Everything below is a fraction of this, so changing it scales the
        // whole profile. Start low and work up. Keep it ≤ TAU_MAX_NM or the clamp bites.
        constexpr float PEAK_TORQUE_NM = 5.0f;

        // Profile shape — fractions of PEAK_TORQUE_NM, indexed by GaitState, so the order must
        // match the enum. The ±1.0 entry is the peak; keep every |fraction| ≤ 1.0.
        constexpr float STATE_TORQUE_FRAC[4] = {
            -0.80f,   // LOADING          heel only: extension, weight acceptance
            -0.45f,   // MID_STANCE       both:      light extension as the body passes over
            +1.00f,   // TERMINAL_STANCE  toe only:  flexion, push-off (the peak)
             0.0f,    // SWING            airborne:  no assist, the leg swings free
        };

        // No swing push-down. The foot is off the ground, so the FSRs say nothing about where the
        // leg actually is — the old push timed itself off the cadence estimate alone, which meant
        // commanding extension torque blind. If the estimate ran ahead of the real swing, or the
        // wearer broke cadence, it pushed against a leg that was still rising. Zero torque in
        // SWING is the honest answer until there's a sensor that can see the limb mid-air.

        // Slew limits on commanded torque so state changes ramp instead of stepping. Absolute —
        // neither scales with PEAK_TORQUE_NM.
        //   TAU_RATE_NM_S         : building torque up. Keep this quick — push-off is short and a
        //                           lazy rise arrives after the wearer needs it. 40 Nm/s crosses
        //                           full scale in ~0.2 s.
        //   TAU_RELEASE_RATE_NM_S : letting torque back off toward zero. Gentler, because the big
        //                           one is toe-off: TERMINAL_STANCE's +PEAK straight to SWING's 0,
        //                           which at the engage rate lands as a 75 ms jolt. 20 Nm/s spreads
        //                           the same drop over ~150 ms — a third of a 440 ms swing, so it
        //                           reads as push-off follow-through and is long gone by heel
        //                           strike. Applies only to normal profile decay: when a gate drops
        //                           (fault, standing, desync) the controller must go limp at the
        //                           full engage rate, so that path ignores this.
        constexpr float TAU_RATE_NM_S         = 40.0f;
        constexpr float TAU_RELEASE_RATE_NM_S = 15.0f;

        // Master scale on top of PEAK_TORQUE_NM (prefer tuning PEAK_TORQUE_NM), and the hard clamp.
        constexpr float GAIN       = 1.0f;   // 0..1
        constexpr float TAU_MAX_NM = 6.0f;   // safety clamp, Nm at the joint
    }

    // IMU / Madgwick filter
    // --------------------------------------------------------------------------------------------
    namespace Imu {
        // Higher = trusts the accelerometer more (settles faster, noisier).
        constexpr float MADGWICK_BETA = 0.2f;
    }

    // Safety — fall detection from the fused pelvis pitch
    // --------------------------------------------------------------------------------------------
    // A fall shows up as the pitch changing fast, ending far from upright, or both. Either
    // condition held for FALL_CONFIRM_SAMPLES cycles latches a fall and safetyTask shuts down.
    namespace Safety {
        // Latches the safe state (motors off, control task suspended) until a power-cycle.
        constexpr bool  FALL_DETECT_ENABLED = true;

        // "Sudden": pelvis pitch rate. Normal walking sways slowly; a topple blows past this.
        constexpr float FALL_RATE_DPS = 150.0f;

        // "Fallen": absolute tilt from upright. Assumes the fused pitch reads ~0 standing —
        // disable if the IMU zero isn't trustworthy.
        constexpr bool  FALL_TILT_ENABLED = true;
        constexpr float FALL_TILT_DEG     = 75.0f;

        // Cycles the condition must hold before latching. 3 at 100 Hz = 30 ms.
        constexpr uint8_t FALL_CONFIRM_SAMPLES = 3;
    }

    // Motor + gearbox — MKS XDrive Mini
    // --------------------------------------------------------------------------------------------
    // The encoder is on the motor shaft, before the gearbox:
    //   joint angle  = motor angle / GEAR_RATIO
    //   joint torque = current * NM_PER_AMP
    namespace Motor {
        constexpr float KT         = 0.23f;   // nameplate Nm/A — reference only
        constexpr float GEAR_RATIO = 15.0f;
        constexpr float GEAR_EFF   = 0.6f;    // efficiency estimate — reference only
        constexpr float I_MAX      = 7.0f;    // drive current limit, A (~27.5 Nm at the joint)

        // Measured 2026-07-09: commanding 1.0 Nm through the old estimate (KT*RATIO*EFF =
        // 2.07 Nm/A) actually produced 1.9 Nm at the joint, so the real scale is 2.07 * 1.9.
        // Re-measure if the motor or gearbox changes.
        constexpr float NM_PER_AMP = 3.93f;

        // Per-leg torque sign, found on the bench so each side assists instead of resisting.
        // If one side ever drives the wrong way, flip only that side and re-check at low GAIN.
        constexpr float DIR_L = +1.0f;
        constexpr float DIR_R = +1.0f;
    }

}
