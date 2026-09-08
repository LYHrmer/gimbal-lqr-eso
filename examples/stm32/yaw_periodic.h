#ifndef STM32_YAW_PERIODIC_H
#define STM32_YAW_PERIODIC_H

#include "yaw_controller.h"

/* HAL-neutral example. All storage is caller-owned. This is not board firmware. */
typedef enum {
    STM32_YAW_OK = 0,
    STM32_YAW_WARMUP,
    STM32_YAW_BAD_ARGUMENT,
    STM32_YAW_BAD_CONFIG,
    STM32_YAW_BAD_CLOCK,
    STM32_YAW_SNAPSHOT_FAILED,
    STM32_YAW_DRIVE_NOT_READY,
    STM32_YAW_CONTROLLER_FAULT,
    STM32_YAW_SUBMIT_FAILED
} Stm32YawResult;

typedef struct {
    YawFeedback feedback;   /* Continuous output-shaft angle and rad/s. */
    YawReference reference;
    uint64_t feedback_source_us;
    uint64_t reference_source_us;
    bool drive_ready; /* Host authorization, decoded drive health and mode. */
} Stm32YawSnapshot;

typedef struct {
    void *user;
    uint64_t (*monotonic_us)(void *user); /* Same clock domain as source stamps. */
    bool (*read_snapshot)(void *user, Stm32YawSnapshot *out);
    /* Nonblocking; copy value/time, validate/encode or publish to group owner.
     * true means accepted, not physically applied. Host monitors CAN delivery. */
    bool (*submit_torque_nm)(void *user, float torque_nm, uint64_t source_us);
    /* Inhibit normal commands, invalidate pending ones, start actual stop path.
     * Never implement this as merely transmitting an encoded torque zero. */
    void (*request_disable)(void *user, Stm32YawResult reason, YawStatus core_status);
} Stm32YawHooks;

typedef struct {
    YawController core;
    YawOutput last_output; /* Diagnostic only; check step result before using. */
    Stm32YawHooks hooks;
    uint64_t previous_tick_us;
    Stm32YawResult latched_result;
    bool initialized;
} Stm32YawPeriodic;

/* Host supplies measured/verified configuration, never simulation defaults.
 * Init/acknowledge/step must be serialized in the same control task.
 * Init and acknowledge do not enable the drive or change its control mode. */
bool stm32_yaw_periodic_init(Stm32YawPeriodic *app, const YawConfig *config,
                             Stm32YawHooks hooks);
/* Host calls only after deliberate fault acknowledgement and stop recovery. */
bool stm32_yaw_periodic_acknowledge(Stm32YawPeriodic *app);
/* Called by a nominal 1 kHz task. Actual elapsed time is passed to the C core. */
Stm32YawResult stm32_yaw_periodic_step(Stm32YawPeriodic *app);

#endif
