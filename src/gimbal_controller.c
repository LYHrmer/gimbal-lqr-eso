#include "gimbal_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "Finite-value checks require disabling fast-math and finite-math-only"
#endif

static double sum_roundoff(double a, double b, double sum)
{
    /* TwoSum residual: even double can lose a tiny positive float margin next
     * to a large endpoint. Retain its sign when sum lands exactly on a float. */
    const double b_rounded = sum - a;
    return (a - (sum - b_rounded)) + (b - b_rounded);
}

bool gimbal_joint_reference_bounds(const GimbalConfig *c, float *low, float *high)
{
    if (low == NULL || high == NULL || low == high || c == NULL || !c->pitch_enabled ||
        !isfinite(c->joint_min_rad) || !isfinite(c->joint_max_rad) ||
        !isfinite(c->joint_margin_rad) || c->joint_margin_rad < 0.0f ||
        c->joint_min_rad >= c->joint_max_rad) {
        if (low != NULL) *low = 0.0f;
        if (high != NULL) *high = 0.0f;
        return false;
    }
    const double exact_low = (double)c->joint_min_rad + (double)c->joint_margin_rad;
    const double exact_high = (double)c->joint_max_rad - (double)c->joint_margin_rad;
    if (exact_low >= exact_high) {
        *low = *high = 0.0f;
        return false;
    }
    /* Round INTO the mathematical inset. A nearest float may otherwise fall
     * outside the requested margin; comparisons and host clamps need the same
     * representable endpoints. A single nextafter is sufficient from nearest. */
    float lower = (float)exact_low;
    float upper = (float)exact_high;
    const double low_roundoff = sum_roundoff((double)c->joint_min_rad,
                                             (double)c->joint_margin_rad, exact_low);
    const double high_roundoff = sum_roundoff((double)c->joint_max_rad,
                                              -(double)c->joint_margin_rad, exact_high);
    if ((double)lower < exact_low || ((double)lower == exact_low && low_roundoff > 0.0))
        lower = nextafterf(lower, INFINITY);
    if ((double)upper > exact_high || ((double)upper == exact_high && high_roundoff < 0.0))
        upper = nextafterf(upper, -INFINITY);
    if (lower >= upper) {
        *low = *high = 0.0f;
        return false;
    }
    *low = lower;
    *high = upper;
    return true;
}

bool gimbal_config_valid(const GimbalConfig *c)
{
    if (c == NULL || !yaw_config_valid(&c->control)) return false;
    if (!isfinite(c->gravity_cos_nm) || !isfinite(c->gravity_sin_nm) ||
        !isfinite(c->joint_min_rad) || !isfinite(c->joint_max_rad) ||
        !isfinite(c->joint_margin_rad)) return false;
    if (!c->pitch_enabled) return c->gravity_cos_nm == 0.0f && c->gravity_sin_nm == 0.0f;
    float low, high;
    return gimbal_joint_reference_bounds(c, &low, &high) &&
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

static YawStatus step(GimbalController *c, const YawFeedback *f,
                      const YawReference *r, const GimbalPose *p,
                      float h, bool external_gravity, float holding_torque_nm,
                      GimbalOutput *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (c == NULL || f == NULL || r == NULL || out == NULL)
        return reject(c, out, YAW_BAD_ARGUMENT);
    if (!c->initialized) return reject(c, out, YAW_BAD_CONFIG);
    if (c->core.fault != YAW_OK) return reject(c, out, c->core.fault);
    if (external_gravity && (!c->config.pitch_enabled ||
        c->config.gravity_cos_nm != 0.0f || c->config.gravity_sin_nm != 0.0f))
        return reject(c, out, YAW_BAD_CONFIG);
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
    float low, high;
    if (!gimbal_joint_reference_bounds(cfg, &low, &high))
        return reject(c, out, YAW_BAD_CONFIG);
    if (p->joint_position_rad < cfg->joint_min_rad || p->joint_position_rad > cfg->joint_max_rad ||
        joint_reference < low || joint_reference > high ||
        (joint_reference <= low && joint_ref_velocity < 0.0) ||
        (joint_reference >= high && joint_ref_velocity > 0.0))
        return reject(c, out, YAW_POSITION_LIMIT);
    const float gravity = external_gravity ? holding_torque_nm :
        cfg->gravity_cos_nm * cosf(p->gravity_angle_rad) +
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

YawStatus gimbal_controller_step(GimbalController *c, const YawFeedback *f,
                                 const YawReference *r, const GimbalPose *p,
                                 float h, GimbalOutput *out)
{
    return step(c, f, r, p, h, false, 0.0f, out);
}

YawStatus gimbal_controller_step_with_gravity(GimbalController *c,
                                 const YawFeedback *f, const YawReference *r,
                                 const GimbalPose *p, float h,
                                 float holding_torque_nm, GimbalOutput *out)
{
    return step(c, f, r, p, h, true, holding_torque_nm, out);
}
