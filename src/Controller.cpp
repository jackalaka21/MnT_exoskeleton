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
    // Each leg's own heel/toe contacts drive its estimator.
    _gait_left.update (fsr_left_heel_contact,  fsr_left_toe_contact);
    _gait_right.update(fsr_right_heel_contact, fsr_right_toe_contact);

    // Publish for the logger / monitor.
    gait_phase_L = _gait_left.phase();
    gait_phase_R = _gait_right.phase();
    gait_state_L = static_cast<uint8_t>(_gait_left.state());
    gait_state_R = static_cast<uint8_t>(_gait_right.state());

    // Standing detector. Both feet loaded at once is brief while walking but continuous while
    // standing, so time it: past STANDING_DOUBLE_SUPPORT_S we cut assist on both legs. Needs
    // sustained contact, so sway and FSR chatter can't fool it — unlike the FSM's silence-based
    // walking() watchdog, which stays as a backstop.
    bool left_loaded  = fsr_left_heel_contact  || fsr_left_toe_contact;
    bool right_loaded = fsr_right_heel_contact || fsr_right_toe_contact;
    if (left_loaded && right_loaded) _double_support_s += Config::Rates::SENSOR_DT;
    else                             _double_support_s = 0.0f;
    _standing = (_double_support_s >= Config::Gait::STANDING_DOUBLE_SUPPORT_S);
    standing_detected = _standing;
}

void Controller::applyAssistiveTorque() {
    // Drain the shared RX FIFO once, then run both legs off the fresh feedback.
    _can.poll();
    if (!_can.armed()) return;

    _controlLeg(MOTOR_NODE_L, _gait_left,  Config::Motor::DIR_L, &motor_angle_L, &motor_vel_L, &tau_cmd_L);
    _controlLeg(MOTOR_NODE_R, _gait_right, Config::Motor::DIR_R, &motor_angle_R, &motor_vel_R, &tau_cmd_R);
}

// Helper Functions
// --------------------------------------------------------------------------------------------
// One leg's torque law: pure assist tracking the gait state. No centering spring — on the exo the
// leg is held by gravity and ground reaction, unlike the bench rig it came from.
void Controller::_controlLeg(uint8_t node, GaitFSM& gait, float dir,
                             volatile float* angle, volatile float* velocity, volatile float* tau_cmd) {
    // Publish the latest angle/velocity (poll() already drained the FIFO this cycle).
    _can.read(node, angle, velocity);

    // Queue the next reads — the drive answers by the next cycle.
    _can.requestAngle(node);
    _can.requestVelocity(node);

    // Torque target: per-state lookup, clamped to ±TAU_MAX_NM. Assist only when all three gates
    // hold, otherwise the target is exactly zero:
    //   !_standing : the wearer isn't stopped with both feet planted.
    //   walking()  : actively striding (standing reads MID_STANCE, which would otherwise command
    //                a constant extension torque). Also covers airborne / bench.
    //   synced()   : the state sequence looks real, so a faulty FSR can't command torque.
    const bool assist_ok = (!_standing && gait.walking() && gait.synced());

    float target = 0.0f;
    if (assist_ok) {
        // dir is this leg's sign (the drive's +current vs our +flexion). Flat per-state target;
        // SWING is zero, so the leg is left free through the airborne phase.
        target = dir * constrain(_assist.compute(gait.state(), Config::Assist::GAIN),
                                 -Config::Assist::TAU_MAX_NM, Config::Assist::TAU_MAX_NM);
    }

    // Slew limit so a state change ramps the torque instead of stepping it. Backing off toward
    // zero uses the gentler release rate, which is what takes the edge off the push-off → swing
    // drop. Only while the gates hold: if assist has been cut we're not shaping a profile any
    // more, we're making the leg safe, and that goes at the full rate.
    const bool releasing = assist_ok && (fabsf(target) < fabsf(*tau_cmd));
    float rate = releasing ? Config::Assist::TAU_RELEASE_RATE_NM_S : Config::Assist::TAU_RATE_NM_S;

    float max_step = rate / Config::Rates::CONTROL_HZ;   // Nm per cycle
    float tau = *tau_cmd + constrain(target - *tau_cmd, -max_step, max_step);

    // Command in Nm — setTorqueNm() converts to motor current.
    *tau_cmd = tau;
    _can.setTorqueNm(node, *tau_cmd);
}
