#include "yaw_controller.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static YawConfig config(void)
{
    return (YawConfig){
        .inertia_kg_m2 = 0.039f, .damping_nm_s_rad = 0.30f,
        .k_position = 10.0f, .k_velocity = 1.0f, .k_integral = 0.0f,
        .integral_limit_nm = 0.5f, .antiwindup_rate_s = 30.0f,
        .coulomb_nm = 0.0f, .coulomb_velocity_rad_s = 0.05f,
        .eso_bandwidth_rad_s = 80.0f, .eso_gain = 1.0f,
        .disturbance_limit_nm = 2.0f, .compensation_limit_nm = 1.0f,
        .compensation_slew_nm_s = 20.0f, .torque_limit_nm = 7.0f,
        .torque_slew_nm_s = 100.0f, .dt_min_s = 0.0005f, .dt_max_s = 0.003f,
        .feedback_timeout_s = 0.005f, .reference_timeout_s = 0.05f,
        .position_min_rad = -10.0f, .position_max_rad = 10.0f,
        .velocity_limit_rad_s = 30.0f, .tracking_error_limit_rad = 3.0f
    };
}

static YawFeedback feedback(void)
{
    return (YawFeedback){.valid = true};
}

static YawReference reference(void)
{
    return (YawReference){.valid = true};
}

static void test_invalid_config(void)
{
    YawController c;
    YawConfig cfg = config();
    assert(yaw_config_valid(&cfg));
    assert(!yaw_config_valid(NULL));
#define BAD_CONFIG(field, value) do { \
    cfg = config(); cfg.field = (value); \
    assert(!yaw_controller_init(&c, &cfg)); \
    assert(c.fault == YAW_BAD_CONFIG && !c.initialized); \
} while (0)
    BAD_CONFIG(inertia_kg_m2, NAN);
    BAD_CONFIG(inertia_kg_m2, 0.0f);
    BAD_CONFIG(torque_limit_nm, -1.0f);
    BAD_CONFIG(torque_slew_nm_s, INFINITY);
    BAD_CONFIG(k_integral, -1.0f);
    BAD_CONFIG(eso_gain, 1.1f);
    BAD_CONFIG(eso_bandwidth_rad_s, 1000.0f);
    BAD_CONFIG(dt_min_s, 0.004f);
    BAD_CONFIG(position_max_rad, -11.0f);
    BAD_CONFIG(velocity_error_filter_tau_s, -0.001f);
    BAD_CONFIG(velocity_error_filter_tau_s, NAN);
    cfg = config(); cfg.k_position = 1.0f; cfg.k_velocity = 0.0f; cfg.k_integral = 10.0f;
    assert(!yaw_config_valid(&cfg)); /* Routh condition for nominal integral loop. */
    cfg = config();
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_init(&c, &c.config));
#undef BAD_CONFIG
}

static void test_timing_and_latched_fault(void)
{
    const float invalid[] = {NAN, INFINITY, -0.001f, 0.0f, 0.00001f, 0.02f, 0.2f};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        YawController c;
        YawConfig cfg = config();
        YawFeedback f = feedback();
        YawReference r = reference();
        YawOutput out;
        assert(yaw_controller_init(&c, &cfg));
        assert(yaw_controller_step(&c, &f, &r, invalid[i], &out) == YAW_BAD_TIMING);
        assert(out.torque_nm == 0.0f && isfinite(out.torque_nm));
        assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_BAD_TIMING);
        yaw_controller_reset(&c);
        assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
        assert(out.torque_nm == 0.0f);
        assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
    }
}

static void test_input_faults(void)
{
    YawController c;
    YawConfig cfg = config();
    YawFeedback f;
    YawReference r;
    YawOutput out;
#define INPUT_FAULT(setup, expected) do { \
    f = feedback(); r = reference(); \
    assert(yaw_controller_init(&c, &cfg)); \
    setup; \
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == (expected)); \
    assert(out.torque_nm == 0.0f && c.last_torque_nm == 0.0f); \
} while (0)
    INPUT_FAULT(r.position_rad = NAN, YAW_BAD_REFERENCE);
    INPUT_FAULT(r.acceleration_rad_s2 = INFINITY, YAW_BAD_REFERENCE);
    INPUT_FAULT(r.valid = false, YAW_BAD_REFERENCE);
    INPUT_FAULT(r.age_s = 0.051f, YAW_STALE_REFERENCE);
    INPUT_FAULT(f.velocity_rad_s = NAN, YAW_BAD_FEEDBACK);
    INPUT_FAULT(f.valid = false, YAW_BAD_FEEDBACK);
    INPUT_FAULT(f.age_s = -0.1f, YAW_BAD_FEEDBACK);
    INPUT_FAULT(f.age_s = 0.006f, YAW_STALE_FEEDBACK);
    INPUT_FAULT(f.applied_torque_valid = true; f.applied_torque_nm = NAN, YAW_BAD_FEEDBACK);
    INPUT_FAULT(f.position_rad = 11.0f, YAW_POSITION_LIMIT);
    INPUT_FAULT(r.position_rad = 11.0f, YAW_POSITION_LIMIT);
    INPUT_FAULT(f.velocity_rad_s = 31.0f, YAW_VELOCITY_LIMIT);
    INPUT_FAULT(r.position_rad = 4.0f, YAW_TRACKING_LIMIT);
#undef INPUT_FAULT
    f = feedback(); r = reference();
    f.applied_torque_nm = NAN; /* Unavailable optional input must be ignored. */
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
    c.integral_nm = NAN;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_NUMERIC_FAULT);
    assert(out.torque_nm == 0.0f);
}

static void test_first_fault_survives_bad_arguments(void)
{
    YawController c;
    YawConfig cfg = config();
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, &f, &r, 0.02f, &out) == YAW_BAD_TIMING);
    assert(yaw_controller_step(&c, NULL, &r, 0.001f, &out) == YAW_BAD_TIMING);
    assert(out.status == YAW_BAD_TIMING && out.torque_nm == 0.0f);
    assert(yaw_controller_step(&c, &f, NULL, 0.001f, &out) == YAW_BAD_TIMING);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, NULL) == YAW_BAD_TIMING);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_BAD_TIMING);
    yaw_controller_reset(&c);
    assert(yaw_controller_step(&c, NULL, &r, 0.001f, &out) == YAW_BAD_ARGUMENT);
    assert(yaw_controller_step(NULL, &f, &r, 0.001f, &out) == YAW_BAD_ARGUMENT);
    cfg.inertia_kg_m2 = 0.0f;
    assert(!yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, NULL, &r, 0.001f, &out) == YAW_BAD_CONFIG);
    assert(out.status == YAW_BAD_CONFIG && out.torque_nm == 0.0f);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, NULL) == YAW_BAD_CONFIG);
}

static void test_derating_and_stop(void)
{
    YawConfig cfg = config();
    cfg.eso_bandwidth_rad_s = 0.0f; cfg.eso_gain = 0.0f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference(); r.position_rad = 0.5f;
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    for (int i = 0; i < 60; ++i)
        assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
    assert(fabsf(out.torque_nm - 5.0f) < 1e-5f);
    assert(yaw_controller_set_torque_limit(&c, 1.0f));
    assert(!yaw_controller_set_torque_limit(&c, NAN));
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
    assert(out.torque_nm <= 1.0f && (out.flags & YAW_TORQUE_LIMITED));
    f.valid = false;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_BAD_FEEDBACK);
    assert(out.torque_nm == 0.0f); /* Fault zero overrides slew. */
    f.valid = true;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_BAD_FEEDBACK);
}

static void test_antiwindup(void)
{
    YawConfig cfg = config();
    cfg.k_position = 1.0f; cfg.k_velocity = 0.0f; cfg.k_integral = 2.0f;
    cfg.integral_limit_nm = 5.0f; cfg.torque_limit_nm = 0.2f;
    cfg.eso_bandwidth_rad_s = 0.0f; cfg.eso_gain = 0.0f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference(); r.position_rad = 0.5f;
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg));
    for (int i = 0; i < 5000; ++i) {
        const YawStatus status = yaw_controller_step(&c, &f, &r, 0.001f, &out);
        assert(status == YAW_OK || status == YAW_WARMUP);
        assert(fabsf(out.torque_nm) <= cfg.torque_limit_nm);
    }
    /* A stalled actuator does not accumulate 5 N.m from 5 s of error. */
    assert(fabsf(c.integral_nm) < 0.4f);
}

static float disturbance_trial(float eso_gain)
{
    YawConfig cfg = config(); cfg.damping_nm_s_rad = 0.0f; cfg.eso_gain = eso_gain;
    YawController c;
    YawReference r = reference();
    YawFeedback f = feedback();
    YawOutput out;
    float position = 0.0f, velocity = 0.0f;
    const float h = 0.001f, disturbance = 0.2f;
    assert(yaw_controller_init(&c, &cfg));
    for (int i = 0; i < 5000; ++i) {
        f.position_rad = position; f.velocity_rad_s = velocity;
        YawStatus status = yaw_controller_step(&c, &f, &r, h, &out);
        assert(status == YAW_OK || status == YAW_WARMUP);
        const float acceleration = (out.torque_nm + disturbance) / cfg.inertia_kg_m2;
        position += h * velocity + 0.5f * h * h * acceleration;
        velocity += h * acceleration;
    }
    assert(fabsf(out.disturbance_nm - disturbance) < 0.01f);
    return fabsf(position);
}

static void test_jitter_and_output_invariants(void)
{
    YawConfig cfg = config(); cfg.torque_limit_nm = 0.5f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    float previous = 0.0f;
    assert(yaw_controller_init(&c, &cfg));
    for (int i = 0; i < 10000; ++i) {
        const float t = (float)i * 0.001f;
        const float h = (i % 2 == 0) ? 0.0007f : 0.0013f;
        r.position_rad = 0.2f * sinf(9.0f * t);
        r.velocity_rad_s = 1.8f * cosf(9.0f * t);
        r.acceleration_rad_s2 = -16.2f * sinf(9.0f * t);
        YawStatus status = yaw_controller_step(&c, &f, &r, h, &out);
        assert(status == YAW_OK || status == YAW_WARMUP);
        assert(isfinite(out.torque_nm) && fabsf(out.torque_nm) <= cfg.torque_limit_nm);
        assert(fabsf(out.torque_nm - previous) <= cfg.torque_slew_nm_s * h + 1e-6f);
        previous = out.torque_nm;
    }
    cfg.k_position = FLT_MAX;
    assert(yaw_controller_init(&c, &cfg));
    r = reference(); r.position_rad = 2.0f;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_NUMERIC_FAULT);
    assert(out.torque_nm == 0.0f);
}

static void test_velocity_error_filter(void)
{
    YawConfig cfg = config();
    cfg.velocity_error_filter_tau_s = 0.002f;
    cfg.eso_bandwidth_rad_s = 0.0f; cfg.eso_gain = 0.0f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    /* A persistent speed error converges to the same DC feedback. Jittered
     * intervals must follow elapsed physical time, not a fixed filter alpha. */
    r.velocity_rad_s = 1.0f;
    float elapsed = 0.0f;
    for (int i = 0; i < 20; ++i) {
        const float h = (i % 2 == 0) ? 0.0007f : 0.0013f;
        elapsed += h;
        assert(yaw_controller_step(&c, &f, &r, h, &out) == YAW_OK);
        assert(fabsf(out.velocity_error_rad_s - (1.0f-expf(-elapsed/0.002f))) < 2e-6f);
    }
    /* The same change in reference and measurement creates no new error. */
    r.velocity_rad_s = 2.0f; f.velocity_rad_s = 1.0f;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
    assert(fabsf(out.velocity_error_rad_s - 1.0f) < 5e-5f);
    f.valid = false;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_BAD_FEEDBACK);
    assert(c.velocity_error_filtered_rad_s == 0.0f && out.torque_nm == 0.0f);
    yaw_controller_reset(&c);
    f.valid = true;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    assert(c.velocity_error_filtered_rad_s == 1.0f);
}

static void test_acceleration_feedforward_observer_history(void)
{
    YawConfig cfg = config();
    cfg.damping_nm_s_rad = 0.0f;
    cfg.torque_slew_nm_s = 10000.0f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    double position = 0.0, velocity = 0.0;
    float previous_command = 0.0f;
    for (int i = 0; i < 400; ++i) {
        const float h = (i % 2 == 0) ? 0.001f : 0.002f;
        /* Independent exact plant: the PREVIOUS total command acted over h.
         * No physical disturbance; commanded acceleration must not become d. */
        const double acceleration = (double)previous_command / cfg.inertia_kg_m2;
        position += velocity * h + 0.5 * acceleration * h * h;
        velocity += acceleration * h;
        f.position_rad = r.position_rad = (float)position;
        f.velocity_rad_s = r.velocity_rad_s = (float)velocity;
        r.acceleration_rad_s2 = (i / 20) % 2 == 0 ? 4.0f : -4.0f;
        assert(yaw_controller_step(&c, &f, &r, h, &out) == YAW_OK);
        assert(fabsf(out.disturbance_nm) < 0.0001f);
        assert(fabsf(out.torque_nm - cfg.inertia_kg_m2 * r.acceleration_rad_s2) < 0.0001f);
        previous_command = out.torque_nm;
    }
    yaw_controller_reset(&c);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    assert(out.torque_nm == 0.0f && c.last_torque_nm == 0.0f);
}

static void test_acceleration_feedforward_limits(void)
{
    YawConfig cfg = config();
    cfg.k_integral = 2.0f;
    cfg.torque_limit_nm = 0.2f;
    cfg.eso_bandwidth_rad_s = cfg.eso_gain = 0.0f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    r.acceleration_rad_s2 = 25.0f;
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg));
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    assert(out.torque_nm == 0.0f);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
    assert((out.flags & YAW_TORQUE_LIMITED) && (out.flags & YAW_SLEW_LIMITED));
    assert(fabsf(out.torque_nm - 0.1f) < 1e-6f);
    /* Zero position error: negative integral is tracking the actual limited
     * total command, including acceleration feedforward, not accumulating e. */
    assert(c.integral_nm < 0.0f);
    float previous = out.torque_nm;
    for (int i = 0; i < 100; ++i) {
        if (i == 50) r.acceleration_rad_s2 = -25.0f;
        assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_OK);
        assert(fabsf(out.torque_nm) <= cfg.torque_limit_nm);
        assert(fabsf(out.torque_nm - previous) <= 0.100001f);
        assert(fabsf(c.integral_nm) <= cfg.integral_limit_nm);
        previous = out.torque_nm;
    }
    /* A stale acceleration packet is not made fresh by the periodic reader. */
    r.age_s = cfg.reference_timeout_s + 0.001f;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_STALE_REFERENCE);
    assert(out.torque_nm == 0.0f && c.integral_nm == 0.0f);
    r.age_s = 0.0f;
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_STALE_REFERENCE);
    yaw_controller_reset(&c);
    assert(yaw_controller_step(&c, &f, &r, 0.001f, &out) == YAW_WARMUP);
    assert(out.torque_nm == 0.0f);
}

static double acceleration_tracking_trial(bool acceleration_enabled, float h)
{
    /* Ideal, noise-free, independent ZOH plant. Only J*a_ref is switched;
     * measured speed feedback and B*v_ref stay identical in both runs. */
    YawConfig cfg = config();
    cfg.eso_bandwidth_rad_s = cfg.eso_gain = 0.0f;
    cfg.torque_slew_nm_s = 10000.0f; /* Isolate tracking from startup limiting. */
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    const double amplitude = 0.08, omega = 4.0 * 3.14159265358979323846;
    const double drag_time = (double)cfg.inertia_kg_m2 / cfg.damping_nm_s_rad;
    const double one_minus_decay = -expm1(-(double)h / drag_time);
    double position = 0.0, velocity = amplitude * omega, sum_squared_error = 0.0;
    int samples = 0;
    assert(yaw_controller_init(&c, &cfg));
    for (int i = 0; (double)i * h < 4.0; ++i) {
        const double t = (double)i * h;
        r.position_rad = (float)(amplitude * sin(omega * t));
        r.velocity_rad_s = (float)(amplitude * omega * cos(omega * t));
        r.acceleration_rad_s2 = acceleration_enabled ?
            (float)(-amplitude * omega * omega * sin(omega * t)) : 0.0f;
        f.position_rad = (float)position;
        f.velocity_rad_s = (float)velocity;
        const YawStatus status = yaw_controller_step(&c, &f, &r, h, &out);
        assert(status == (i == 0 ? YAW_WARMUP : YAW_OK));
        assert(!(out.flags & (YAW_TORQUE_LIMITED | YAW_SLEW_LIMITED)));
        if (t >= 0.5) {
            const double error = (double)r.position_rad - position;
            sum_squared_error += error * error;
            ++samples;
        }
        const double equilibrium_velocity = (double)out.torque_nm / cfg.damping_nm_s_rad;
        position += equilibrium_velocity * h +
            (velocity - equilibrium_velocity) * drag_time * one_minus_decay;
        velocity += (equilibrium_velocity - velocity) * one_minus_decay;
    }
    assert(samples > 0);
    return sqrt(sum_squared_error / samples);
}

static void test_acceleration_tracking(void)
{
    const float periods[] = {0.001f, 0.002f};
    for (unsigned i = 0; i < sizeof(periods) / sizeof(periods[0]); ++i) {
        const double off = acceleration_tracking_trial(false, periods[i]);
        const double on = acceleration_tracking_trial(true, periods[i]);
        /* A wiring regression, not a hardware performance acceptance gate. */
        assert(off > 0.001 && on < 0.1 * off);
        printf("ideal acceleration A/B: dt=%.3f s, RMSE off=%.8f on=%.8f rad\n",
               (double)periods[i], off, on);
    }
}

int main(void)
{
    test_invalid_config();
    test_velocity_error_filter();
    test_timing_and_latched_fault();
    test_input_faults();
    test_first_fault_survives_bad_arguments();
    test_derating_and_stop();
    test_antiwindup();
    const float baseline_error = disturbance_trial(0.0f);
    const float eso_error = disturbance_trial(1.0f);
    assert(baseline_error > 0.015f && eso_error < 0.002f);
    assert(eso_error < 0.15f * baseline_error);
    test_jitter_and_output_invariants();
    test_acceleration_feedforward_observer_history();
    test_acceleration_feedforward_limits();
    test_acceleration_tracking();
    printf("yaw_controller: all test groups passed; synthetic load hold error %.7f -> %.7f rad\n",
           (double)baseline_error, (double)eso_error);
    return 0;
}
