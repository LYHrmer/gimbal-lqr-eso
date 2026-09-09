#include "gimbal_periodic.h"

#include <stddef.h>
#include <string.h>

#ifdef __FAST_MATH__
#error "gimbal_periodic must preserve the controller finite-value contract"
#endif

static Stm32GimbalResult stop(Stm32GimbalPeriodic *app, Stm32GimbalResult reason)
{
    app->latched_result = reason;
    app->last_output.control.torque_nm = 0.0f; /* Software value, not a stop. */
    app->last_output.gravity_feedforward_nm = 0.0f;
    if (app->hooks.request_disable != NULL)
        app->hooks.request_disable(app->hooks.user, reason,
                                    app->last_output.control.status);
    return reason;
}

bool stm32_gimbal_periodic_init(Stm32GimbalPeriodic *app,
                                const GimbalConfig *config,
                                Stm32GimbalHooks hooks)
{
    if (app == NULL) return false;
    /* Permit reinitialization with &app->core.config. Hooks are passed by value. */
    const bool valid_config = gimbal_config_valid(config);
    GimbalConfig copy = {0};
    if (valid_config) copy = *config;
    memset(app, 0, sizeof(*app));
    app->hooks = hooks;
    if (hooks.monotonic_us == NULL || hooks.read_snapshot == NULL ||
        hooks.submit_torque_nm == NULL || hooks.request_disable == NULL) {
        (void)stop(app, STM32_GIMBAL_BAD_ARGUMENT);
        return false;
    }
    if (!valid_config || !gimbal_controller_init(&app->core, &copy)) {
        app->last_output.control.status = YAW_BAD_CONFIG;
        (void)stop(app, STM32_GIMBAL_BAD_CONFIG);
        return false;
    }
    app->previous_tick_us = hooks.monotonic_us(hooks.user);
    app->initialized = true;
    return true;
}

bool stm32_gimbal_periodic_acknowledge(Stm32GimbalPeriodic *app)
{
    if (app == NULL || !app->initialized) return false;
    gimbal_controller_reset(&app->core);
    memset(&app->last_output, 0, sizeof(app->last_output));
    app->previous_tick_us = app->hooks.monotonic_us(app->hooks.user);
    app->latched_result = STM32_GIMBAL_OK;
    return true;
}

Stm32GimbalResult stm32_gimbal_periodic_step(Stm32GimbalPeriodic *app)
{
    if (app == NULL) return STM32_GIMBAL_BAD_ARGUMENT;
    if (app->latched_result != STM32_GIMBAL_OK) return app->latched_result;
    if (!app->initialized) return stop(app, STM32_GIMBAL_BAD_ARGUMENT);
    Stm32GimbalSnapshot sample = {0};
    if (!app->hooks.read_snapshot(app->hooks.user, &sample))
        return stop(app, STM32_GIMBAL_SNAPSHOT_FAILED);
    /* Read after snapshot so newly received data cannot appear newer than now.
     * Snapshot coherence and oldest-source selection are host responsibilities. */
    const uint64_t now_us = app->hooks.monotonic_us(app->hooks.user);
    const bool use_pose = app->core.config.pitch_enabled;
    if (now_us <= app->previous_tick_us || sample.feedback_source_us > now_us ||
        sample.reference_source_us > now_us ||
        (use_pose && sample.pose_source_us > now_us))
        return stop(app, STM32_GIMBAL_BAD_CLOCK);
    const float dt_s = (float)((double)(now_us - app->previous_tick_us) * 1e-6);
    app->previous_tick_us = now_us;
    sample.feedback.age_s = (float)((double)(now_us - sample.feedback_source_us) * 1e-6);
    sample.reference.age_s = (float)((double)(now_us - sample.reference_source_us) * 1e-6);
    if (use_pose)
        sample.pose.age_s = (float)((double)(now_us - sample.pose_source_us) * 1e-6);
    /* A current/torque sample is not the mean over the previous interval. */
    sample.feedback.applied_torque_valid = false;
    sample.feedback.applied_torque_nm = 0.0f;
    if (!sample.drive_ready) return stop(app, STM32_GIMBAL_DRIVE_NOT_READY);
    const YawStatus status = gimbal_controller_step(&app->core, &sample.feedback,
        &sample.reference, use_pose ? &sample.pose : NULL, dt_s, &app->last_output);
    if (status != YAW_OK && status != YAW_WARMUP)
        return stop(app, STM32_GIMBAL_CONTROLLER_FAULT);
    if (!app->hooks.submit_torque_nm(app->hooks.user,
                                    app->last_output.control.torque_nm, now_us))
        return stop(app, STM32_GIMBAL_SUBMIT_FAILED);
    return status == YAW_WARMUP ? STM32_GIMBAL_WARMUP : STM32_GIMBAL_OK;
}
