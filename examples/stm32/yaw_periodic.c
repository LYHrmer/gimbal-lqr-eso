#include "yaw_periodic.h"

#include <stddef.h>
#include <string.h>

#ifdef __FAST_MATH__
#error "yaw_periodic must preserve the controller finite-value contract"
#endif

static Stm32YawResult stop(Stm32YawPeriodic *app, Stm32YawResult reason)
{
    app->latched_result = reason;
    app->last_output.torque_nm = 0.0f; /* Software value; never a physical stop. */
    if (app->hooks.request_disable != NULL)
        app->hooks.request_disable(app->hooks.user, reason, app->last_output.status);
    return reason;
}

bool stm32_yaw_periodic_init(Stm32YawPeriodic *app, const YawConfig *config,
                             Stm32YawHooks hooks)
{
    if (app == NULL) return false;
    /* Permit reinitialization with &app->core.config. Hooks are passed by value. */
    const bool valid_config = yaw_config_valid(config);
    YawConfig copy = {0};
    if (valid_config) copy = *config;
    memset(app, 0, sizeof(*app));
    app->hooks = hooks;
    if (hooks.monotonic_us == NULL || hooks.read_snapshot == NULL ||
        hooks.submit_torque_nm == NULL || hooks.request_disable == NULL) {
        (void)stop(app, STM32_YAW_BAD_ARGUMENT);
        return false;
    }
    if (!valid_config || !yaw_controller_init(&app->core, &copy)) {
        app->last_output.status = YAW_BAD_CONFIG;
        (void)stop(app, STM32_YAW_BAD_CONFIG);
        return false;
    }
    app->previous_tick_us = hooks.monotonic_us(hooks.user);
    app->initialized = true;
    return true;
}

bool stm32_yaw_periodic_acknowledge(Stm32YawPeriodic *app)
{
    if (app == NULL || !app->initialized) return false;
    yaw_controller_reset(&app->core);
    memset(&app->last_output, 0, sizeof(app->last_output));
    app->previous_tick_us = app->hooks.monotonic_us(app->hooks.user);
    app->latched_result = STM32_YAW_OK;
    return true;
}

Stm32YawResult stm32_yaw_periodic_step(Stm32YawPeriodic *app)
{
    if (app == NULL) return STM32_YAW_BAD_ARGUMENT;
    if (app->latched_result != STM32_YAW_OK) return app->latched_result;
    if (!app->initialized) return stop(app, STM32_YAW_BAD_ARGUMENT);
    Stm32YawSnapshot sample = {0};
    if (!app->hooks.read_snapshot(app->hooks.user, &sample))
        return stop(app, STM32_YAW_SNAPSHOT_FAILED);
    /* Read after snapshot so a concurrently received frame cannot appear newer
     * than the sampled current time. Snapshot itself must be internally coherent. */
    const uint64_t now_us = app->hooks.monotonic_us(app->hooks.user);
    if (now_us <= app->previous_tick_us || sample.feedback_source_us > now_us ||
        sample.reference_source_us > now_us)
        return stop(app, STM32_YAW_BAD_CLOCK);
    const float dt_s = (float)((double)(now_us - app->previous_tick_us) * 1e-6);
    app->previous_tick_us = now_us;
    sample.feedback.age_s = (float)((double)(now_us - sample.feedback_source_us) * 1e-6);
    sample.reference.age_s = (float)((double)(now_us - sample.reference_source_us) * 1e-6);
    /* A single current/torque feedback sample is not a previous-interval mean. */
    sample.feedback.applied_torque_valid = false;
    sample.feedback.applied_torque_nm = 0.0f;
    if (!sample.drive_ready) return stop(app, STM32_YAW_DRIVE_NOT_READY);
    const YawStatus status = yaw_controller_step(&app->core, &sample.feedback,
        &sample.reference, dt_s, &app->last_output);
    if (status != YAW_OK && status != YAW_WARMUP)
        return stop(app, STM32_YAW_CONTROLLER_FAULT);
    if (!app->hooks.submit_torque_nm(app->hooks.user, app->last_output.torque_nm, now_us))
        return stop(app, STM32_YAW_SUBMIT_FAILED);
    return status == YAW_WARMUP ? STM32_YAW_WARMUP : STM32_YAW_OK;
}
