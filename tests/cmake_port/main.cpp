/* Host-only C++17 consumer: synthetic fixtures, never hardware defaults. */
#include "gimbal_periodic.h"
#include "gimbal_coordinates.h"
#if PORT_MOTOR_GM6020
#include "gm6020.h"
#else
#include "dm_mit.h"
#endif
#include <cassert>

int main()
{
    GimbalConfig cfg{};
    cfg.control.inertia_kg_m2 = 0.03f;
    cfg.control.damping_nm_s_rad = 0.1f;
    cfg.control.k_position = 5.0f;
    cfg.control.k_velocity = 0.5f;
    cfg.control.coulomb_velocity_rad_s = 0.1f;
    cfg.control.disturbance_limit_nm = 1.0f;
    cfg.control.compensation_limit_nm = 0.5f;
    cfg.control.compensation_slew_nm_s = 50.0f;
    cfg.control.torque_limit_nm = 1.0f;
    cfg.control.torque_slew_nm_s = 100.0f;
    cfg.control.dt_min_s = 0.0005f;
    cfg.control.dt_max_s = 0.005f;
    cfg.control.feedback_timeout_s = 0.01f;
    cfg.control.reference_timeout_s = 0.05f;
    cfg.control.position_min_rad = -2.0f;
    cfg.control.position_max_rad = 2.0f;
    cfg.control.velocity_limit_rad_s = 10.0f;
    cfg.control.tracking_error_limit_rad = 1.0f;
    cfg.pitch_enabled = true;
    cfg.joint_min_rad = -1.0f;
    cfg.joint_max_rad = 1.0f;
    cfg.joint_margin_rad = 0.1f;
    GimbalController axis{};
    assert(gimbal_controller_init(&axis, &cfg));
    YawFeedback feedback{};
    YawReference reference{};
    GimbalPose pose{};
    feedback.valid = reference.valid = pose.valid = true;
    GimbalOutput output{};
    assert(gimbal_controller_step_with_gravity(&axis, &feedback, &reference, &pose,
                                               0.001f, 0.3f, &output) == YAW_WARMUP);
    assert(gimbal_controller_step_with_gravity(&axis, &feedback, &reference, &pose,
                                               0.001f, 0.3f, &output) == YAW_OK);
    assert(output.control.torque_nm > 0.0f && output.control.torque_nm <= 0.100001f);
    /* Opposite motor direction is an output mapping, not a negative inertia. */
#if PORT_MOTOR_GM6020
    gm6020_config_t drive{0.741, 1.5, 1.0, true};
    gm6020_current_word_t word{};
    assert(gm6020_torque_to_word(&drive, -output.control.torque_nm, &word) == GM6020_OK);
    assert(word.valid && word.raw < 0);
    const int16_t slots[4] = {word.raw, 123, 0, 0};
    gm6020_command_t command{};
    assert(gm6020_pack_current_group(0x1FE, slots, 4, true, &command) == GM6020_OK);
    assert(command.data[2] == 0 && command.data[3] == 123);
#else
    dm_mit_config_t drive{};
    dm_mit_command_t command{};
    assert(dm_mit_encode_torque(&drive, output.control.torque_nm, &command) != DM_MIT_OK);
    assert(!command.valid); /* Unconfigured drive must not produce a command. */
#endif
    Stm32GimbalPeriodic periodic{};
    Stm32GimbalHooks hooks{};
    assert(!stm32_gimbal_periodic_init(&periodic, &cfg, hooks));
    float nearby = 0.0f;
    assert(gimbal_angle_near(0.1f, 0.0f, &nearby));
    assert(nearby > 0.0f);
    return 0;
}
