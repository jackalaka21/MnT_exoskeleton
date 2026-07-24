#include "Controller.h"

#include "Config.h"
#include "Pin.h"
#include "VolatileData.h"

// Constructor
// --------------------------------------------------------------------------------------------
Controller::Controller(MotorCAN& can)
    : _can(can)
    , _gait_left("left",  Config::Rates::SENSOR_DT)
    , _gait_right("right", Config::Rates::SENSOR_DT)
    , _assist("assist") {}

// Public Methods
// --------------------------------------------------------------------------------------------
void Controller::updateGaitPhase() {
    // FSR-only: heel/toe contact flags drive the estimator (no hip-velocity gate). Each leg gets
    // its own foot contacts; the phase stays synced to the wearer's cadence.
    _gait_left.update (fsr_left_heel_contact,  fsr_left_toe_contact);
    _gait_right.update(fsr_right_heel_contact, fsr_right_toe_contact);

    // Publish the gait state for the logger / monitor.
    gait_phase_L = _gait_left.phase();
    gait_phase_R = _gait_right.phase();
    gait_state_L = static_cast<uint8_t>(_gait_left.state());
    gait_state_R = static_cast<uint8_t>(_gait_right.state());
}

void Controller::applyAssistiveTorque() {
    // Drain the shared RX FIFO once, then service both legs from the fresh feedback.
    _can.poll();
    if (!_can.armed()) return;

    _controlLeg(MOTOR_NODE_L, _gait_left,  Config::Motor::DIR_L, &motor_angle_L, &motor_vel_L, &tau_cmd_L);
    _controlLeg(MOTOR_NODE_R, _gait_right, Config::Motor::DIR_R, &motor_angle_R, &motor_vel_R, &tau_cmd_R);
}

// Helper Functions
// --------------------------------------------------------------------------------------------
// One leg's torque law — REAL HARDWARE: pure assistive torque tracking the gait phase.
//   assist — the AssistiveTorque profile maps this leg's gait phase (from its FSR-driven GaitFSM)
//            to an OpenExo-style flexion/extension torque.
// No centering spring: on the exo the leg is anchored by gravity and ground reaction, so the bench
// spring (which held a free arm to a home angle) is gone. The command is clamped to a safety
// backstop above the profile's own peak (max designed |torque| ≈ 5 Nm, hardware ≈ 27 Nm).
void Controller::_controlLeg(uint8_t node, GaitFSM& gait, float dir,
                             volatile float* angle, volatile float* velocity, volatile float* tau_cmd) {
    // Publish latest angle/velocity (poll() already drained the shared RX FIFO this cycle)
    _can.read(node, angle, velocity);

    // Queue next feedback requests (drive answers by the next cycle)
    _can.requestAngle(node);
    _can.requestVelocity(node);

    // CONTACT-DRIVEN assist: the target torque is a direct lookup on the current foot-contact
    // state. SAFETY GATES — assist only when BOTH hold, else the target is exactly zero:
    //   walking() : actively striding (standing on both feet reads MID_STANCE; without this that
    //               would command a constant extension torque; also covers airborne / bench).
    //   synced()  : the gait sequence looks real — a faulty FSR that produces an implausible state
    //               transition drops sync, so it cannot command an unexpected torque.
    float target = 0.0f;
    if (gait.walking() && gait.synced()) {
        // dir carries this leg's torque sign (the drive's +current vs our +flexion convention).
        // In SWING the torque comes from the cadence-timed push-down (swingProgress); elsewhere
        // it is the flat per-state target.
        target = dir * constrain(_assist.compute(gait.state(), gait.swingProgress(), Config::Assist::GAIN),
                                 -Config::Assist::TAU_MAX_NM, Config::Assist::TAU_MAX_NM);
    }

    // Slew-rate limit toward the target so a state change ramps the torque instead of stepping it.
    float max_step = Config::Assist::TAU_RATE_NM_S / Config::Rates::CONTROL_HZ;   // Nm per cycle
    float tau = *tau_cmd + constrain(target - *tau_cmd, -max_step, max_step);

    // Command joint torque in Nm — converted to a motor current inside setTorqueNm().
    *tau_cmd = tau;
    _can.setTorqueNm(node, *tau_cmd);
}
