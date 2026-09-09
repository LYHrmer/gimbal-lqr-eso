#include "yaw_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifdef __FAST_MATH__
#error "yaw_controller requires finite-value checks; compile without -ffast-math"
#endif

static float clampf(float x, float low, float high)
{
    return fminf(fmaxf(x, low), high);
}

bool yaw_config_valid(const YawConfig *c)
{
    if (c == NULL) return false;
    const float values[] = {
        c->inertia_kg_m2, c->damping_nm_s_rad, c->k_position, c->k_velocity,
        c->k_integral, c->integral_limit_nm, c->antiwindup_rate_s,
        c->coulomb_nm, c->coulomb_velocity_rad_s, c->eso_bandwidth_rad_s,
        c->eso_gain, c->disturbance_limit_nm, c->compensation_limit_nm,
        c->compensation_slew_nm_s, c->torque_limit_nm, c->torque_slew_nm_s,
        c->dt_min_s, c->dt_max_s, c->feedback_timeout_s, c->reference_timeout_s,
        c->position_min_rad, c->position_max_rad, c->velocity_limit_rad_s,
        c->tracking_error_limit_rad, c->velocity_error_filter_tau_s
    };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
        if (!isfinite(values[i])) return false;
    /* Necessary nominal continuous-time stability condition only. Discrete
     * poles, delay, saturation and the full ESO loop still need validation. */
    const double damping = (double)c->damping_nm_s_rad + (double)c->k_velocity;
    if (damping <= 0.0 || (c->k_integral > 0.0f &&
        damping * (double)c->k_position <= (double)c->inertia_kg_m2 * (double)c->k_integral))
        return false;
    return c->inertia_kg_m2 >= 1e-7f && c->damping_nm_s_rad >= 0.0f &&
        c->k_position > 0.0f && c->k_velocity >= 0.0f && c->k_integral >= 0.0f &&
        c->integral_limit_nm >= 0.0f && c->antiwindup_rate_s >= 0.0f &&
        (c->k_integral == 0.0f ||
         (c->integral_limit_nm > 0.0f && c->antiwindup_rate_s > 0.0f)) &&
        c->coulomb_nm >= 0.0f && c->coulomb_velocity_rad_s > 0.0f &&
        c->eso_bandwidth_rad_s >= 0.0f && c->eso_gain >= 0.0f && c->eso_gain <= 1.0f &&
        c->disturbance_limit_nm > 0.0f && c->compensation_limit_nm > 0.0f &&
        c->compensation_slew_nm_s > 0.0f &&
        (c->eso_bandwidth_rad_s > 0.0f || c->eso_gain == 0.0f) &&
        c->torque_limit_nm > 0.0f && c->torque_slew_nm_s > 0.0f &&
        c->dt_min_s >= 1e-6f && c->dt_max_s >= c->dt_min_s && c->dt_max_s <= 0.05f &&
        c->eso_bandwidth_rad_s * c->dt_max_s <= 1.0f &&
        c->feedback_timeout_s > 0.0f && c->reference_timeout_s > 0.0f &&
        c->position_min_rad < c->position_max_rad &&
        c->velocity_limit_rad_s > 0.0f && c->tracking_error_limit_rad > 0.0f &&
        c->velocity_error_filter_tau_s >= 0.0f;
}

static void clear_history(YawController *c)
{
    c->observer_position_rad = 0.0f;
    c->observer_velocity_rad_s = 0.0f;
    c->observer_disturbance_nm = 0.0f;
    c->integral_nm = 0.0f;
    c->compensation_nm = 0.0f;
    c->last_torque_nm = 0.0f;
    c->last_velocity_rad_s = 0.0f;
    c->velocity_error_filtered_rad_s = 0.0f;
    c->ready = false;
}

bool yaw_controller_init(YawController *c, const YawConfig *config)
{
    if (c == NULL) return false;
    /* Copy before clearing, so init(c, &c->config) is well-defined. */
    if (!yaw_config_valid(config)) {
        memset(c, 0, sizeof(*c));
        c->fault = YAW_BAD_CONFIG;
        return false;
    }
    const YawConfig copy = *config;
    memset(c, 0, sizeof(*c));
    c->config = copy;
    c->initialized = true;
    c->fault = YAW_OK;
    return true;
}

void yaw_controller_reset(YawController *c)
{
    if (c == NULL) return;
    clear_history(c);
    c->fault = c->initialized ? YAW_OK : YAW_BAD_CONFIG;
}

bool yaw_controller_set_torque_limit(YawController *c, float limit_nm)
{
    if (c == NULL || !c->initialized || !isfinite(limit_nm) || limit_nm <= 0.0f)
        return false;
    c->config.torque_limit_nm = limit_nm;
    return true;
}

static YawStatus fault(YawController *c, YawOutput *out, YawStatus reason)
{
    if (c != NULL) {
        clear_history(c);
        c->fault = reason;
    }
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->status = reason;
    }
    return reason;
}

YawStatus yaw_controller_latch_fault(YawController *c, YawStatus reason, YawOutput *out)
{
    if (c == NULL) return fault(c, out, YAW_BAD_ARGUMENT);
    if (!c->initialized) return fault(c, out, YAW_BAD_CONFIG);
    if (c->fault != YAW_OK) return fault(c, out, c->fault);
    if (reason < YAW_BAD_ARGUMENT || reason > YAW_NUMERIC_FAULT)
        reason = YAW_BAD_ARGUMENT;
    return fault(c, out, reason);
}

static float friction(const YawConfig *c, float velocity)
{
    return c->coulomb_nm * tanhf(velocity / c->coulomb_velocity_rad_s);
}

static bool finite_history(const YawController *c)
{
    return isfinite(c->observer_position_rad) && isfinite(c->observer_velocity_rad_s) &&
        isfinite(c->observer_disturbance_nm) && isfinite(c->integral_nm) &&
        isfinite(c->compensation_nm) && isfinite(c->last_torque_nm) &&
        isfinite(c->last_velocity_rad_s) && isfinite(c->velocity_error_filtered_rad_s);
}

/* Predict/correct ESO for the residual double-integrator model.
 * Nominal drag is an explicitly known, held input from the last measurement.
 * z_d is residual disturbance torque (N.m), not total acceleration: therefore
 * nominal viscous/Coulomb feedforward is not compensated a second time.
 * For fixed h, constant held inputs and no clamps, all three error poles are
 * p=exp(-w*h). This is NOT a proof for the full nonlinear saturated loop.
 */
static bool update_observer(YawController *c, const YawFeedback *f, float h,
                            float previous_load_nm,
                            uint32_t *flags)
{
    const YawConfig *cfg = &c->config;
    if (cfg->eso_bandwidth_rad_s == 0.0f) {
        c->observer_position_rad = f->position_rad;
        c->observer_velocity_rad_s = f->velocity_rad_s;
        c->observer_disturbance_nm = 0.0f;
        return true;
    }
    const float u = f->applied_torque_valid ? f->applied_torque_nm : c->last_torque_nm;
    const float net = u - cfg->damping_nm_s_rad * c->last_velocity_rad_s -
        friction(cfg, c->last_velocity_rad_s) + c->observer_disturbance_nm - previous_load_nm;
    const float acceleration = net / cfg->inertia_kg_m2;
    const float pred_p = c->observer_position_rad + h * c->observer_velocity_rad_s +
        0.5f * h * h * acceleration;
    const float pred_v = c->observer_velocity_rad_s + h * acceleration;
    const float innovation = f->position_rad - pred_p;
    const float wh = cfg->eso_bandwidth_rad_s * h;
    const float q = -expm1f(-wh); /* 1-exp(-wh), accurate for small wh. */
    const float l1 = -expm1f(-3.0f * wh);
    const float l2 = 1.5f * q * q * (2.0f - q) / h;
    const float l3 = cfg->inertia_kg_m2 * q * q * q / (h * h);
    const float next_d = c->observer_disturbance_nm + l3 * innovation;
    c->observer_position_rad = pred_p + l1 * innovation;
    c->observer_velocity_rad_s = pred_v + l2 * innovation;
    if (!isfinite(next_d) || !isfinite(c->observer_position_rad) ||
        !isfinite(c->observer_velocity_rad_s)) return false;
    c->observer_disturbance_nm = clampf(next_d, -cfg->disturbance_limit_nm,
                                       cfg->disturbance_limit_nm);
    if (next_d != c->observer_disturbance_nm) *flags |= YAW_DISTURBANCE_LIMITED;
    return true;
}

YawStatus yaw_controller_step(YawController *c, const YawFeedback *f,
                              const YawReference *r, float h, YawOutput *out)
{
    return yaw_controller_step_with_load(c, f, r, h, 0.0f, 0.0f, out);
}

YawStatus yaw_controller_step_with_load(YawController *c, const YawFeedback *f,
                              const YawReference *r, float h, float current_load_nm,
                              float previous_load_nm, YawOutput *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (c == NULL || f == NULL || r == NULL || out == NULL)
        return fault(c, out, YAW_BAD_ARGUMENT);
    if (!c->initialized) return fault(c, out, YAW_BAD_CONFIG);
    if (c->fault != YAW_OK) return fault(c, out, c->fault);
    if (!isfinite(current_load_nm) || !isfinite(previous_load_nm))
        return fault(c, out, YAW_NUMERIC_FAULT);
    const YawConfig *cfg = &c->config;
    if (!isfinite(h) || h < cfg->dt_min_s || h > cfg->dt_max_s)
        return fault(c, out, YAW_BAD_TIMING);
    if (!f->valid || !isfinite(f->position_rad) || !isfinite(f->velocity_rad_s) ||
        !isfinite(f->age_s) || f->age_s < 0.0f ||
        (f->applied_torque_valid && !isfinite(f->applied_torque_nm)))
        return fault(c, out, YAW_BAD_FEEDBACK);
    if (f->age_s > cfg->feedback_timeout_s)
        return fault(c, out, YAW_STALE_FEEDBACK);
    if (!r->valid || !isfinite(r->position_rad) || !isfinite(r->velocity_rad_s) ||
        !isfinite(r->acceleration_rad_s2) || !isfinite(r->age_s) || r->age_s < 0.0f)
        return fault(c, out, YAW_BAD_REFERENCE);
    if (r->age_s > cfg->reference_timeout_s)
        return fault(c, out, YAW_STALE_REFERENCE);
    if (f->position_rad < cfg->position_min_rad || f->position_rad > cfg->position_max_rad ||
        r->position_rad < cfg->position_min_rad || r->position_rad > cfg->position_max_rad)
        return fault(c, out, YAW_POSITION_LIMIT);
    if (fabsf(f->velocity_rad_s) > cfg->velocity_limit_rad_s ||
        fabsf(r->velocity_rad_s) > cfg->velocity_limit_rad_s)
        return fault(c, out, YAW_VELOCITY_LIMIT);
    const float ep = r->position_rad - f->position_rad;
    const float raw_ev = r->velocity_rad_s - f->velocity_rad_s;
    if (!isfinite(ep) || !isfinite(raw_ev)) return fault(c, out, YAW_NUMERIC_FAULT);
    if (fabsf(ep) > cfg->tracking_error_limit_rad)
        return fault(c, out, YAW_TRACKING_LIMIT);
    if (!finite_history(c)) return fault(c, out, YAW_NUMERIC_FAULT);
    if (!c->ready) {
        c->observer_position_rad = f->position_rad;
        c->observer_velocity_rad_s = f->velocity_rad_s;
        c->last_velocity_rad_s = f->velocity_rad_s;
        c->velocity_error_filtered_rad_s = raw_ev;
        c->ready = true;
        out->status = YAW_WARMUP;
        return YAW_WARMUP;
    }
    if (!update_observer(c, f, h, previous_load_nm, &out->flags))
        return fault(c, out, YAW_NUMERIC_FAULT);

    /* Filter the velocity error, so the reference and measurement receive the
     * same dynamics. Optional filtering trades noise attenuation for feedback
     * phase lag; it is not an unconditional stability/performance improvement. */
    if (cfg->velocity_error_filter_tau_s == 0.0f) {
        c->velocity_error_filtered_rad_s = raw_ev;
    } else {
        const float alpha = -expm1f(-h / cfg->velocity_error_filter_tau_s);
        /* A double convex combination avoids overflowing raw_ev - old when
         * both finite float endpoints have opposite signs. */
        c->velocity_error_filtered_rad_s = (float)((1.0 - (double)alpha) *
            (double)c->velocity_error_filtered_rad_s + (double)alpha * (double)raw_ev);
    }
    const float ev = c->velocity_error_filtered_rad_s;
    if (!isfinite(ev)) return fault(c, out, YAW_NUMERIC_FAULT);

    const float target_comp = -cfg->eso_gain * c->observer_disturbance_nm;
    const float bounded_comp = clampf(target_comp, -cfg->compensation_limit_nm,
                                      cfg->compensation_limit_nm);
    const float comp_delta = cfg->compensation_slew_nm_s * h;
    c->compensation_nm = cfg->eso_gain == 0.0f ? 0.0f :
        clampf(bounded_comp, c->compensation_nm - comp_delta, c->compensation_nm + comp_delta);
    if (target_comp != c->compensation_nm) out->flags |= YAW_COMPENSATION_LIMITED;

    const float i_unbounded = cfg->k_integral == 0.0f ? 0.0f :
        c->integral_nm + cfg->k_integral * ep * h;
    if (!isfinite(i_unbounded)) return fault(c, out, YAW_NUMERIC_FAULT);
    const float i_used = clampf(i_unbounded, -cfg->integral_limit_nm, cfg->integral_limit_nm);
    if (i_unbounded != i_used) out->flags |= YAW_INTEGRAL_LIMITED;
    out->feedforward_nm = cfg->inertia_kg_m2 * r->acceleration_rad_s2 +
        cfg->damping_nm_s_rad * r->velocity_rad_s + friction(cfg, r->velocity_rad_s) +
        current_load_nm;
    out->feedback_nm = cfg->k_position * ep + cfg->k_velocity * ev;
    out->integral_nm = i_used;
    out->compensation_nm = c->compensation_nm;
    out->disturbance_nm = c->observer_disturbance_nm;
    out->position_error_rad = ep;
    out->velocity_error_rad_s = ev;
    const float raw = out->feedforward_nm + out->feedback_nm + i_used + c->compensation_nm;
    if (!isfinite(raw) || !isfinite(out->feedforward_nm) || !isfinite(out->feedback_nm))
        return fault(c, out, YAW_NUMERIC_FAULT);
    out->unconstrained_torque_nm = raw;
    const float bounded = clampf(raw, -cfg->torque_limit_nm, cfg->torque_limit_nm);
    const float delta = cfg->torque_slew_nm_s * h;
    const float slewed = clampf(bounded, c->last_torque_nm - delta, c->last_torque_nm + delta);
    /* Final hard limit wins, including abrupt online derating and fault recovery. */
    const float command = clampf(slewed, -cfg->torque_limit_nm, cfg->torque_limit_nm);
    if (bounded != raw || command != slewed) out->flags |= YAW_TORQUE_LIMITED;
    if (slewed != bounded) out->flags |= YAW_SLEW_LIMITED;

    /* Back-calculation uses the final executable command, including slew.
     * Exponential tracking weight stays in [0,1] under valid timing.
     * output.integral_nm is the term used THIS step; state holds NEXT step's term. */
    if (cfg->k_integral > 0.0f) {
        const float weight = -expm1f(-cfg->antiwindup_rate_s * h);
        const float next_i = i_used + weight * (command - raw);
        if (!isfinite(next_i)) return fault(c, out, YAW_NUMERIC_FAULT);
        c->integral_nm = clampf(next_i, -cfg->integral_limit_nm, cfg->integral_limit_nm);
        if (next_i != c->integral_nm) out->flags |= YAW_INTEGRAL_LIMITED;
    } else c->integral_nm = 0.0f;
    c->last_torque_nm = command;
    c->last_velocity_rad_s = f->velocity_rad_s;
    if (!isfinite(command) || !finite_history(c))
        return fault(c, out, YAW_NUMERIC_FAULT);
    out->torque_nm = command;
    out->status = YAW_OK;
    return YAW_OK;
}

const char *yaw_status_string(YawStatus status)
{
    switch (status) {
    case YAW_OK: return "ok";
    case YAW_WARMUP: return "warmup";
    case YAW_BAD_ARGUMENT: return "bad_argument";
    case YAW_BAD_CONFIG: return "bad_config";
    case YAW_BAD_TIMING: return "bad_timing";
    case YAW_BAD_FEEDBACK: return "bad_feedback";
    case YAW_STALE_FEEDBACK: return "stale_feedback";
    case YAW_BAD_REFERENCE: return "bad_reference";
    case YAW_STALE_REFERENCE: return "stale_reference";
    case YAW_POSITION_LIMIT: return "position_limit";
    case YAW_VELOCITY_LIMIT: return "velocity_limit";
    case YAW_TRACKING_LIMIT: return "tracking_limit";
    case YAW_NUMERIC_FAULT: return "numeric_fault";
    default: return "unknown";
    }
}
