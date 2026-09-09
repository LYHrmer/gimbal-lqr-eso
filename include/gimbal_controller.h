#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include "yaw_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Single-axis adapter. Allocate separate instances for yaw and pitch.
 * The legacy Yaw* types express SI axis quantities and remain source-compatible.
 * Pitch gravity uses world inclination; joint limits use encoder-relative angle.
 * Positive angle, angular velocity and motor torque must share one direction. */
typedef struct {
    YawConfig control;
    float gravity_cos_nm;       /* Required holding torque A*cos(theta_g)+B*sin(theta_g). */
    float gravity_sin_nm;
    float joint_min_rad;        /* Verified mechanical boundaries, relative encoder. */
    float joint_max_rad;
    float joint_margin_rad;     /* References must stay inside this inset. */
    bool pitch_enabled;        /* false: exact legacy yaw path; pose may be NULL. */
} GimbalConfig;

typedef struct {
    float joint_position_rad;
    float joint_velocity_rad_s;
    float gravity_angle_rad;   /* Inclination to horizontal, not encoder zero. */
    float age_s;               /* Age of the oldest source used in this pose. */
    bool valid;
} GimbalPose;

typedef struct {
    YawOutput control;
    float gravity_feedforward_nm; /* Term used in this step, zero on warmup/fault. */
    float joint_reference_rad;
} GimbalOutput;

typedef struct {
    YawController core;
    GimbalConfig config;
    float last_gravity_nm;
    bool initialized;
} GimbalController;

bool gimbal_config_valid(const GimbalConfig *config);
bool gimbal_controller_init(GimbalController *controller, const GimbalConfig *config);
void gimbal_controller_reset(GimbalController *controller);
bool gimbal_controller_set_torque_limit(GimbalController *controller, float limit_nm);
/* Pitch assumes the planar relation q_joint_ref=q_joint+q_ref-q_feedback.
 * Supply continuous angles in a coherent snapshot. General tilted-axis/roll
 * geometry requires a host coordinate transform, not this planar adapter.
 * Invalid/stale pose and joint/reference violations latch a core fault. Limits
 * reject references; they are not a braking trajectory or a mechanical stop.
 * Fault/warmup output is zero: a gravity-loaded axis can fall. Host owns brake,
 * counterbalance/support, inhibit/disable and deliberate fault recovery. */
YawStatus gimbal_controller_step(GimbalController *controller,
                                 const YawFeedback *feedback,
                                 const YawReference *reference,
                                 const GimbalPose *pose, float dt_s,
                                 GimbalOutput *output);

#ifdef __cplusplus
}
#endif
#endif
