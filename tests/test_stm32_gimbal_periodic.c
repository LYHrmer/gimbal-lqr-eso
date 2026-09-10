#include "gimbal_periodic.h"
#include "../variants/dm4310/pitch_simulation_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint64_t now;
    Stm32GimbalSnapshot sample;
    unsigned submitted, disabled;
    bool read_ok, submit_ok;
    float torque;
    YawStatus fault;
    Stm32GimbalResult disable_reason;
} Mock;

static uint64_t clock_us(void *user) { return ((Mock *)user)->now; }
static bool snapshot(void *user, Stm32GimbalSnapshot *out)
{
    *out = ((Mock *)user)->sample;
    return ((Mock *)user)->read_ok;
}
static bool submit(void *user, float torque, uint64_t stamp)
{
    Mock *m = user;
    assert(stamp == m->now);
    ++m->submitted;
    m->torque = torque;
    return m->submit_ok;
}
static void disable(void *user, Stm32GimbalResult reason, YawStatus status)
{
    Mock *m = user;
    assert(reason != STM32_GIMBAL_OK);
    ++m->disabled;
    m->fault = status;
    m->disable_reason = reason;
}
static void init(Stm32GimbalPeriodic *app, Mock *m, bool pitch)
{
    memset(m, 0, sizeof(*m));
    m->now = 100000u;
    m->read_ok = m->submit_ok = m->sample.drive_ready = true;
    m->sample.feedback.valid = m->sample.reference.valid = m->sample.pose.valid = true;
    GimbalConfig config = dm4310_pitch_simulation_config();
    if (!pitch) {
        config.pitch_enabled = false;
        config.gravity_cos_nm = config.gravity_sin_nm = 0.0f;
    }
    const Stm32GimbalHooks hooks = {m, clock_us, snapshot, submit, disable};
    assert(stm32_gimbal_periodic_init(app, &config, hooks));
}
static void advance(Mock *m, uint64_t delta)
{
    m->now += delta;
    m->sample.feedback_source_us = m->sample.reference_source_us = m->now;
    m->sample.pose_source_us = m->now;
}

static void start_with_nonzero_torque(Stm32GimbalPeriodic *app, Mock *m)
{
    init(app, m, true);
    advance(m, 1000u);
    assert(stm32_gimbal_periodic_step(app) == STM32_GIMBAL_WARMUP);
    advance(m, 1000u);
    assert(stm32_gimbal_periodic_step(app) == STM32_GIMBAL_OK);
    assert(m->torque > 0.0f && m->submitted == 2u && m->disabled == 0u);
}

static void test_active_fault_latch_and_recovery(void)
{
    enum { FUTURE_FEEDBACK, FUTURE_REFERENCE, FUTURE_POSE, CLOCK_STALLED,
           CLOCK_REVERSED, READ_FAILED, DRIVE_WITHDRAWN, SUBMIT_FAILED };
    const struct {
        int injection;
        Stm32GimbalResult expected;
        unsigned attempted_submissions;
    } cases[] = {
        {FUTURE_FEEDBACK, STM32_GIMBAL_BAD_CLOCK, 0u},
        {FUTURE_REFERENCE, STM32_GIMBAL_BAD_CLOCK, 0u},
        {FUTURE_POSE, STM32_GIMBAL_BAD_CLOCK, 0u},
        {CLOCK_STALLED, STM32_GIMBAL_BAD_CLOCK, 0u},
        {CLOCK_REVERSED, STM32_GIMBAL_BAD_CLOCK, 0u},
        {READ_FAILED, STM32_GIMBAL_SNAPSHOT_FAILED, 0u},
        {DRIVE_WITHDRAWN, STM32_GIMBAL_DRIVE_NOT_READY, 0u},
        {SUBMIT_FAILED, STM32_GIMBAL_SUBMIT_FAILED, 1u},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        Stm32GimbalPeriodic app;
        Mock m;
        start_with_nonzero_torque(&app, &m);
        const unsigned before = m.submitted;
        advance(&m, 1000u);
        switch (cases[i].injection) {
        case FUTURE_FEEDBACK: ++m.sample.feedback_source_us; break;
        case FUTURE_REFERENCE: ++m.sample.reference_source_us; break;
        case FUTURE_POSE: ++m.sample.pose_source_us; break;
        case CLOCK_STALLED:
        case CLOCK_REVERSED:
            m.now = app.previous_tick_us - (cases[i].injection == CLOCK_REVERSED ? 1u : 0u);
            advance(&m, 0u); /* Keep sources current; isolate the loop-clock fault. */
            break;
        case READ_FAILED: m.read_ok = false; break;
        case DRIVE_WITHDRAWN: m.sample.drive_ready = false; break;
        case SUBMIT_FAILED: m.submit_ok = false; break;
        default: assert(false); break;
        }
        const unsigned stopped_submissions = before + cases[i].attempted_submissions;
        assert(stm32_gimbal_periodic_step(&app) == cases[i].expected);
        assert(m.submitted == stopped_submissions && m.disabled == 1u);
        assert(m.disable_reason == cases[i].expected);
        assert(app.last_output.control.torque_nm == 0.0f);
        assert(app.last_output.gravity_feedforward_nm == 0.0f);

        /* Fresh, ready inputs alone must not resume commands or repeat disable. */
        m.read_ok = m.submit_ok = m.sample.drive_ready = true;
        advance(&m, 1000u);
        assert(stm32_gimbal_periodic_step(&app) == cases[i].expected);
        assert(m.submitted == stopped_submissions && m.disabled == 1u);
        assert(stm32_gimbal_periodic_acknowledge(&app));
        assert(m.submitted == stopped_submissions && m.disabled == 1u);
        advance(&m, 1000u);
        assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_WARMUP);
        assert(m.submitted == stopped_submissions + 1u && m.torque == 0.0f);
        assert(app.last_output.gravity_feedforward_nm == 0.0f && m.disabled == 1u);
    }
}

static void test_init_alias_and_missing_hooks(void)
{
    Stm32GimbalPeriodic app;
    Mock m;
    start_with_nonzero_torque(&app, &m);
    const GimbalConfig saved = app.core.config;
    const Stm32GimbalHooks saved_hooks = app.hooks;
    m.sample.drive_ready = false; /* Reconfigure with the host drive inhibited. */
    assert(stm32_gimbal_periodic_init(&app, &app.core.config, app.hooks));
    assert(app.core.config.control.inertia_kg_m2 == saved.control.inertia_kg_m2);
    assert(app.core.config.control.torque_limit_nm == saved.control.torque_limit_nm);
    assert(app.core.config.gravity_cos_nm == saved.gravity_cos_nm);
    assert(app.core.config.gravity_sin_nm == saved.gravity_sin_nm);
    assert(app.core.config.joint_min_rad == saved.joint_min_rad);
    assert(app.core.config.joint_max_rad == saved.joint_max_rad);
    assert(app.core.config.joint_margin_rad == saved.joint_margin_rad);
    assert(app.core.config.pitch_enabled == saved.pitch_enabled);
    assert(app.hooks.user == saved_hooks.user && app.hooks.monotonic_us == saved_hooks.monotonic_us);
    assert(app.hooks.read_snapshot == saved_hooks.read_snapshot);
    assert(app.hooks.submit_torque_nm == saved_hooks.submit_torque_nm);
    assert(app.hooks.request_disable == saved_hooks.request_disable);
    assert(m.submitted == 2u && m.disabled == 0u); /* Init invokes no output hooks. */
    m.sample.drive_ready = true;
    advance(&m, 1000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_WARMUP);
    assert(m.torque == 0.0f && m.submitted == 3u);

    for (unsigned missing = 0u; missing < 4u; ++missing) {
        init(&app, &m, true);
        Stm32GimbalHooks hooks = app.hooks;
        switch (missing) {
        case 0u: hooks.monotonic_us = NULL; break;
        case 1u: hooks.read_snapshot = NULL; break;
        case 2u: hooks.submit_torque_nm = NULL; break;
        case 3u: hooks.request_disable = NULL; break;
        default: assert(false); break;
        }
        assert(!stm32_gimbal_periodic_init(&app, &saved, hooks));
        assert(!app.initialized && app.latched_result == STM32_GIMBAL_BAD_ARGUMENT);
        assert(m.submitted == 0u);
        /* A missing stop callback cannot be invoked, but init still fails shut. */
        assert(m.disabled == (missing == 3u ? 0u : 1u));
        assert(!stm32_gimbal_periodic_acknowledge(&app));
        advance(&m, 1000u);
        assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_BAD_ARGUMENT);
        assert(m.submitted == 0u && m.disabled == (missing == 3u ? 0u : 1u));
    }
}

int main(void)
{
    Stm32GimbalPeriodic app;
    Mock m;
    init(&app, &m, true);
    advance(&m, 1000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_WARMUP);
    assert(m.torque == 0.0f && m.submitted == 1u);
    advance(&m, 700u);
    m.sample.feedback.applied_torque_valid = true;
    m.sample.feedback.applied_torque_nm = NAN; /* Instantaneous sample is ignored. */
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_OK);
    assert(fabsf(m.torque - 0.07f) < 1e-6f); /* Gravity request obeys actual-dt slew. */
    assert(fabsf(app.last_output.gravity_feedforward_nm - 0.45f) < 1e-6f);

    /* Fresh primary IMU data cannot hide an old encoder/base pose component. */
    advance(&m, 1000u);
    m.sample.pose_source_us -= 6000u;
    m.sample.pose.age_s = 0.0f; /* Adapter must derive age from the source stamp. */
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_CONTROLLER_FAULT);
    assert(m.fault == YAW_STALE_FEEDBACK && m.disabled == 1u && m.submitted == 2u);
    assert(app.last_output.control.torque_nm == 0.0f);
    advance(&m, 1000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_CONTROLLER_FAULT);
    assert(m.disabled == 1u && m.submitted == 2u);
    assert(stm32_gimbal_periodic_acknowledge(&app));
    advance(&m, 1000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_WARMUP);

    init(&app, &m, true); advance(&m, 3000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_CONTROLLER_FAULT);
    assert(m.fault == YAW_BAD_TIMING);
    init(&app, &m, true); advance(&m, 1000u);
    m.sample.pose.gravity_angle_rad = NAN;
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_CONTROLLER_FAULT);
    assert(m.fault == YAW_BAD_FEEDBACK);

    /* A yaw instance has no dependency on an unused pitch pose or its clock. */
    init(&app, &m, false); advance(&m, 1000u);
    m.sample.pose.valid = false;
    m.sample.pose.gravity_angle_rad = NAN;
    m.sample.pose_source_us = UINT64_MAX;
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_WARMUP);
    assert(!stm32_gimbal_periodic_init(&app, NULL, app.hooks));
    assert(!stm32_gimbal_periodic_acknowledge(&app));
    test_active_fault_latch_and_recovery();
    test_init_alias_and_missing_hooks();
    puts("stm32_gimbal_periodic: independent pose age, gravity, timing and stop checks passed");
    return 0;
}
