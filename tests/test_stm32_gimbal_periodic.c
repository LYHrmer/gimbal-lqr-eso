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
    bool submit_ok;
    float torque;
    YawStatus fault;
} Mock;

static uint64_t clock_us(void *user) { return ((Mock *)user)->now; }
static bool snapshot(void *user, Stm32GimbalSnapshot *out)
{
    *out = ((Mock *)user)->sample;
    return true;
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
}
static void init(Stm32GimbalPeriodic *app, Mock *m, bool pitch)
{
    memset(m, 0, sizeof(*m));
    m->now = 100000u;
    m->submit_ok = m->sample.drive_ready = true;
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

    init(&app, &m, true); advance(&m, 1000u);
    ++m.sample.pose_source_us;
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_BAD_CLOCK);
    assert(m.disabled == 1u && m.submitted == 0u);
    init(&app, &m, true); advance(&m, 3000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_CONTROLLER_FAULT);
    assert(m.fault == YAW_BAD_TIMING);
    init(&app, &m, true); advance(&m, 1000u);
    m.sample.pose.gravity_angle_rad = NAN;
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_CONTROLLER_FAULT);
    assert(m.fault == YAW_BAD_FEEDBACK);
    init(&app, &m, true); advance(&m, 1000u);
    m.submit_ok = false;
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_SUBMIT_FAILED);
    advance(&m, 1000u);
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_SUBMIT_FAILED);
    assert(m.submitted == 1u && m.disabled == 1u);

    /* A yaw instance has no dependency on an unused pitch pose or its clock. */
    init(&app, &m, false); advance(&m, 1000u);
    m.sample.pose.valid = false;
    m.sample.pose.gravity_angle_rad = NAN;
    m.sample.pose_source_us = UINT64_MAX;
    assert(stm32_gimbal_periodic_step(&app) == STM32_GIMBAL_WARMUP);
    assert(!stm32_gimbal_periodic_init(&app, NULL, app.hooks));
    assert(!stm32_gimbal_periodic_acknowledge(&app));
    puts("stm32_gimbal_periodic: independent pose age, gravity, timing and stop checks passed");
    return 0;
}
