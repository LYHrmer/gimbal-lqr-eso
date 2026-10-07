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
/* Pitch reference bounds, rounded inward to representable float endpoints.
 * Host clamps and endpoint velocity checks must use these same values instead
 * of recomputing joint_min + margin / joint_max - margin with float arithmetic.
 * Only pitch_enabled and joint geometry are validated here; use
 * gimbal_config_valid() for the complete controller/gravity configuration.
 * Returns false if disabled, invalid, or fewer than two distinct floats fit
 * inside the inset. On failure, non-NULL outputs are zeroed. Outputs must be
 * distinct and must not alias config. Physical hard limits remain unchanged. */
bool gimbal_joint_reference_bounds(const GimbalConfig *config,
                                    float *low_rad, float *high_rad);
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
/* Custom pitch gravity model, with the SAME pose/joint/fault/limiter checks.
 * Requires pitch_enabled=true and both built-in gravity coefficients zero.
 * holding_torque_nm is the signed motor torque needed to balance gravity:
 * plant J*a = motor_torque - drag - holding_torque + residual.
 * Pass the CURRENT model value; the adapter retains the previous value for ESO.
 * Include all model input source ages in pose.age_s; retain valid pose fields.
 * No extra gravity may be added after this call. This is not a PRBS/excitation
 * input or a measured torque. Reset on deliberate control/model transitions.
 * Existing struct layouts and the built-in-model step remain unchanged. */
YawStatus gimbal_controller_step_with_gravity(GimbalController *controller,
                                 const YawFeedback *feedback,
                                 const YawReference *reference,
                                 const GimbalPose *pose, float dt_s,
                                 float holding_torque_nm, GimbalOutput *output);

#ifdef __cplusplus
}
#endif
#endif
