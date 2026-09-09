#include "gimbal_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifdef __FAST_MATH__
#error "gimbal_controller requires finite-value checks; compile without -ffast-math"
#endif

bool gimbal_config_valid(const GimbalConfig *c)
{
    if (c == NULL || !yaw_config_valid(&c->control)) return false;
    if (!isfinite(c->gravity_cos_nm) || !isfinite(c->gravity_sin_nm) ||
        !isfinite(c->joint_min_rad) || !isfinite(c->joint_max_rad) ||
        !isfinite(c->joint_margin_rad)) return false;
    if (!c->pitch_enabled) return c->gravity_cos_nm == 0.0f && c->gravity_sin_nm == 0.0f;
    return c->joint_margin_rad >= 0.0f && c->joint_min_rad < c->joint_max_rad &&
        (double)c->joint_min_rad + (double)c->joint_margin_rad <
        (double)c->joint_max_rad - (double)c->joint_margin_rad &&
        isfinite(hypotf(c->gravity_cos_nm, c->gravity_sin_nm));
}

bool gimbal_controller_init(GimbalController *c, const GimbalConfig *config)
{
    if (c == NULL) return false;
    if (!gimbal_config_valid(config)) {
        memset(c, 0, sizeof(*c));
        c->core.fault = YAW_BAD_CONFIG;
        return false;
    }
    const GimbalConfig copy = *config;
    memset(c, 0, sizeof(*c));
    c->config = copy;
    c->initialized = yaw_controller_init(&c->core, &copy.control);
    return c->initialized;
}

void gimbal_controller_reset(GimbalController *c)
{
    if (c == NULL) return;
    yaw_controller_reset(&c->core);
    c->last_gravity_nm = 0.0f;
}

bool gimbal_controller_set_torque_limit(GimbalController *c, float limit_nm)
{
    if (c == NULL || !c->initialized ||
        !yaw_controller_set_torque_limit(&c->core, limit_nm)) return false;
    c->config.control.torque_limit_nm = limit_nm;
    return true;
}

static YawStatus reject(GimbalController *c, GimbalOutput *out, YawStatus reason)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (c != NULL) c->last_gravity_nm = 0.0f;
    return yaw_controller_latch_fault(c == NULL ? NULL : &c->core, reason,
                                      out == NULL ? NULL : &out->control);
}

YawStatus gimbal_controller_step(GimbalController *c, const YawFeedback *f,
                                 const YawReference *r, const GimbalPose *p,
                                 float h, GimbalOutput *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (c == NULL || f == NULL || r == NULL || out == NULL)
        return reject(c, out, YAW_BAD_ARGUMENT);
    if (!c->initialized) return reject(c, out, YAW_BAD_CONFIG);
    if (c->core.fault != YAW_OK) return reject(c, out, c->core.fault);
    if (!c->config.pitch_enabled)
        return yaw_controller_step(&c->core, f, r, h, &out->control);
    if (p == NULL || !p->valid || !isfinite(p->joint_position_rad) ||
        !isfinite(p->joint_velocity_rad_s) || !isfinite(p->gravity_angle_rad) ||
        !isfinite(p->age_s) || p->age_s < 0.0f)
        return reject(c, out, YAW_BAD_FEEDBACK);
    if (p->age_s > c->core.config.feedback_timeout_s)
        return reject(c, out, YAW_STALE_FEEDBACK);
    /* Let the core diagnose invalid primary samples before using them in the
     * frame transform. Do not turn NaN input into an apparent joint-limit hit. */
    if (!f->valid || !isfinite(f->position_rad) || !isfinite(f->velocity_rad_s) ||
        !isfinite(f->age_s) || f->age_s < 0.0f ||
        (f->applied_torque_valid && !isfinite(f->applied_torque_nm)))
        return reject(c, out, YAW_BAD_FEEDBACK);
    if (!r->valid || !isfinite(r->position_rad) || !isfinite(r->velocity_rad_s) ||
        !isfinite(r->acceleration_rad_s2) || !isfinite(r->age_s) || r->age_s < 0.0f)
        return reject(c, out, YAW_BAD_REFERENCE);
    const GimbalConfig *cfg = &c->config;
    const double joint_reference = (double)p->joint_position_rad +
        (double)r->position_rad - (double)f->position_rad;
    const double joint_ref_velocity = (double)p->joint_velocity_rad_s +
        (double)r->velocity_rad_s - (double)f->velocity_rad_s;
    const double low = (double)cfg->joint_min_rad + (double)cfg->joint_margin_rad;
    const double high = (double)cfg->joint_max_rad - (double)cfg->joint_margin_rad;
    if (p->joint_position_rad < cfg->joint_min_rad || p->joint_position_rad > cfg->joint_max_rad ||
        joint_reference < low || joint_reference > high ||
        (joint_reference <= low && joint_ref_velocity < 0.0) ||
        (joint_reference >= high && joint_ref_velocity > 0.0))
        return reject(c, out, YAW_POSITION_LIMIT);
    const float gravity = cfg->gravity_cos_nm * cosf(p->gravity_angle_rad) +
        cfg->gravity_sin_nm * sinf(p->gravity_angle_rad);
    if (!isfinite(gravity) || !isfinite(c->last_gravity_nm))
        return reject(c, out, YAW_NUMERIC_FAULT);
    const YawStatus status = yaw_controller_step_with_load(&c->core, f, r, h,
        gravity, c->last_gravity_nm, &out->control);
    if (status == YAW_OK || status == YAW_WARMUP) {
        c->last_gravity_nm = gravity;
        out->joint_reference_rad = (float)joint_reference;
        if (status == YAW_OK) out->gravity_feedforward_nm = gravity;
    } else c->last_gravity_nm = 0.0f;
    return status;
}
