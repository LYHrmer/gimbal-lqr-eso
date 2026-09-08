#include "yaw_periodic.h"
#include "../variants/dm4310/simulation_config.h" /* Host test fixture only. */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint64_t now_us;
    Stm32YawSnapshot sample;
    bool read_ok;
    bool submit_ok;
    unsigned submitted;
    unsigned disabled;
    float submitted_torque;
    uint64_t submitted_time_us;
    Stm32YawResult disable_reason;
    YawStatus disable_core_status;
} Mock;

static uint64_t clock_us(void *user) { return ((Mock *)user)->now_us; }
static bool read_snapshot(void *user, Stm32YawSnapshot *out)
{
    const Mock *mock = user;
    *out = mock->sample;
    return mock->read_ok;
}
static bool submit(void *user, float torque, uint64_t time_us)
{
    Mock *mock = user;
    ++mock->submitted;
    mock->submitted_torque = torque;
    mock->submitted_time_us = time_us;
    return mock->submit_ok;
}
static void disable(void *user, Stm32YawResult reason, YawStatus status)
{
    Mock *mock = user;
    ++mock->disabled;
    mock->disable_reason = reason;
    mock->disable_core_status = status;
}
static void initialize(Stm32YawPeriodic *app, Mock *mock)
{
    memset(mock, 0, sizeof(*mock));
    mock->now_us = 100000u;
    mock->read_ok = mock->submit_ok = true;
    mock->sample.feedback.valid = mock->sample.reference.valid = true;
    mock->sample.drive_ready = true;
    const Stm32YawHooks hooks = {mock, clock_us, read_snapshot, submit, disable};
    const YawConfig config = dm4310_simulation_config();
    assert(stm32_yaw_periodic_init(app, &config, hooks));
}
static void advance(Mock *mock, uint64_t elapsed_us)
{
    mock->now_us += elapsed_us;
    mock->sample.feedback_source_us = mock->now_us;
    mock->sample.reference_source_us = mock->now_us;
}

int main(void)
{
    Stm32YawPeriodic app;
    Mock mock;
    initialize(&app, &mock);
    advance(&mock, 1000u);
    /* The example deliberately ignores instantaneous applied-torque samples. */
    mock.sample.feedback.applied_torque_valid = true;
    mock.sample.feedback.applied_torque_nm = NAN;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_WARMUP);
    assert(mock.submitted_torque == 0.0f && mock.submitted == 1u);
    assert(mock.submitted_time_us == mock.now_us);
    mock.sample.reference.position_rad = 0.1f;
    advance(&mock, 700u);
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_OK);
    assert(fabsf(mock.submitted_torque - 0.07f) < 1e-6f); /* Actual 0.7 ms slew. */
    advance(&mock, 3000u);
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_CONTROLLER_FAULT);
    assert(mock.disable_core_status == YAW_BAD_TIMING && mock.disabled == 1u);
    assert(mock.submitted == 2u && app.last_output.torque_nm == 0.0f);
    advance(&mock, 1000u);
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_CONTROLLER_FAULT);
    assert(mock.submitted == 2u && mock.disabled == 1u);
    assert(stm32_yaw_periodic_acknowledge(&app));
    advance(&mock, 1000u);
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_WARMUP);
    assert(mock.submitted == 3u && mock.submitted_torque == 0.0f);

    initialize(&app, &mock); advance(&mock, 1000u);
    mock.sample.feedback_source_us -= 6000u;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_CONTROLLER_FAULT);
    assert(mock.disable_core_status == YAW_STALE_FEEDBACK && mock.submitted == 0u);
    initialize(&app, &mock); advance(&mock, 1000u);
    mock.sample.reference.position_rad = NAN;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_CONTROLLER_FAULT);
    assert(mock.disable_core_status == YAW_BAD_REFERENCE);
    initialize(&app, &mock); advance(&mock, 1000u);
    ++mock.sample.feedback_source_us;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_BAD_CLOCK);
    initialize(&app, &mock);
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_BAD_CLOCK);
    initialize(&app, &mock); advance(&mock, 1000u);
    mock.sample.drive_ready = false;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_DRIVE_NOT_READY);
    initialize(&app, &mock); advance(&mock, 1000u);
    mock.read_ok = false;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_SNAPSHOT_FAILED);
    initialize(&app, &mock); advance(&mock, 1000u);
    mock.submit_ok = false;
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_SUBMIT_FAILED);
    assert(mock.disabled == 1u && mock.disable_reason == STM32_YAW_SUBMIT_FAILED);
    advance(&mock, 1000u);
    assert(stm32_yaw_periodic_step(&app) == STM32_YAW_SUBMIT_FAILED);
    assert(mock.submitted == 1u);
    assert(!stm32_yaw_periodic_init(&app, NULL, app.hooks));
    assert(!stm32_yaw_periodic_acknowledge(&app));
    puts("stm32_yaw_periodic: time, snapshot, stop, latch and reset checks passed");
    return 0;
}
