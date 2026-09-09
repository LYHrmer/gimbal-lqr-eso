#ifndef STM32_GIMBAL_PERIODIC_H
#define STM32_GIMBAL_PERIODIC_H

#include "gimbal_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HAL-neutral example. Allocate one instance per axis. This is not firmware. */
typedef enum {
    STM32_GIMBAL_OK = 0,
    STM32_GIMBAL_WARMUP,
    STM32_GIMBAL_BAD_ARGUMENT,
    STM32_GIMBAL_BAD_CONFIG,
    STM32_GIMBAL_BAD_CLOCK,
    STM32_GIMBAL_SNAPSHOT_FAILED,
    STM32_GIMBAL_DRIVE_NOT_READY,
    STM32_GIMBAL_CONTROLLER_FAULT,
    STM32_GIMBAL_SUBMIT_FAILED
} Stm32GimbalResult;

typedef struct {
    YawFeedback feedback;   /* Selected control coordinate: rad and rad/s. */
    YawReference reference;
    GimbalPose pose;        /* Encoder joint coordinate and gravity inclination. */
    uint64_t feedback_source_us;
    uint64_t reference_source_us;
    /* Host stores the OLDEST source stamp used to construct pose, including
     * encoder, inclination and any base-pose transform. Ignored for yaw. */
    uint64_t pose_source_us;
    bool drive_ready;       /* Host authorization, decoded drive health and mode. */
} Stm32GimbalSnapshot;

typedef struct {
    void *user;
    uint64_t (*monotonic_us)(void *user); /* Same domain as all source stamps. */
    bool (*read_snapshot)(void *user, Stm32GimbalSnapshot *out);
    /* Nonblocking; copy value/time, validate/encode or publish to group owner.
     * true means accepted, not physically applied. Host monitors CAN delivery. */
    bool (*submit_torque_nm)(void *user, float torque_nm, uint64_t source_us);
    /* Inhibit normal commands, invalidate pending ones, start actual stop path.
     * Zero torque does not hold an unbalanced pitch. Host owns support/brake. */
    void (*request_disable)(void *user, Stm32GimbalResult reason,
                            YawStatus core_status);
} Stm32GimbalHooks;

typedef struct {
    GimbalController core;
    GimbalOutput last_output; /* Diagnostic only; check step result before use. */
    Stm32GimbalHooks hooks;
    uint64_t previous_tick_us;
    Stm32GimbalResult latched_result;
    bool initialized;
} Stm32GimbalPeriodic;

/* Use measured/verified per-axis configuration, never simulation defaults.
 * Init/acknowledge/step must be serialized in the same control task.
 * Init and acknowledge do not enable a drive or change its control mode. */
bool stm32_gimbal_periodic_init(Stm32GimbalPeriodic *app,
                                const GimbalConfig *config,
                                Stm32GimbalHooks hooks);
/* Host calls only after deliberate fault acknowledgement and stop recovery. */
bool stm32_gimbal_periodic_acknowledge(Stm32GimbalPeriodic *app);
/* Called by a nominal 1 kHz task. Actual elapsed time reaches the C core. */
Stm32GimbalResult stm32_gimbal_periodic_step(Stm32GimbalPeriodic *app);

#ifdef __cplusplus
}
#endif
#endif
