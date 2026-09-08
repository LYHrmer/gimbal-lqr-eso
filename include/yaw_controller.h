#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All angles are continuous, unwrapped output-shaft radians. No implicit wrap.
 * Torque is N.m, time is seconds. No allocation, I/O, driver, or auto-enable.
 * Configure once before init; use reset after a deliberate fault acknowledgement.
 * Do not compile with -ffast-math: finite-value checks are part of the contract. */
typedef struct {
    float inertia_kg_m2;
    float damping_nm_s_rad;
    float k_position;
    float k_velocity;
    float k_integral;
    float integral_limit_nm;
    float antiwindup_rate_s;
    float coulomb_nm;
    float coulomb_velocity_rad_s;
    float eso_bandwidth_rad_s;       /* 0 disables observer. */
    float eso_gain;                  /* [0,1]; 0 observes without compensating. */
    float disturbance_limit_nm;
    float compensation_limit_nm;
    float compensation_slew_nm_s;
    float torque_limit_nm;           /* Application limit, NOT protocol TMAX. */
    float torque_slew_nm_s;
    float dt_min_s;
    float dt_max_s;
    float feedback_timeout_s;
    float reference_timeout_s;
    float position_min_rad;
    float position_max_rad;
    float velocity_limit_rad_s;
    float tracking_error_limit_rad;
    float velocity_error_filter_tau_s; /* 0 bypasses; >0 filters ref - measured velocity. */
} YawConfig;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float age_s;                     /* Computed by host from a monotonic clock. */
    float applied_torque_nm;         /* Optional mean torque over PREVIOUS interval. */
    bool valid;
    bool applied_torque_valid;       /* false: observer uses last limited command. */
} YawFeedback;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float acceleration_rad_s2;
    float age_s;                     /* Timestamp of source, not of loop readout. */
    bool valid;
} YawReference;

typedef enum {
    YAW_OK = 0,
    YAW_WARMUP = 1,
    YAW_BAD_ARGUMENT = 2,
    YAW_BAD_CONFIG = 3,
    YAW_BAD_TIMING = 4,
    YAW_BAD_FEEDBACK = 5,
    YAW_STALE_FEEDBACK = 6,
    YAW_BAD_REFERENCE = 7,
    YAW_STALE_REFERENCE = 8,
    YAW_POSITION_LIMIT = 9,
    YAW_VELOCITY_LIMIT = 10,
    YAW_TRACKING_LIMIT = 11,
    YAW_NUMERIC_FAULT = 12
} YawStatus;

enum {
    YAW_TORQUE_LIMITED = 1u << 0,
    YAW_SLEW_LIMITED = 1u << 1,
    YAW_INTEGRAL_LIMITED = 1u << 2,
    YAW_DISTURBANCE_LIMITED = 1u << 3,
    YAW_COMPENSATION_LIMITED = 1u << 4
};

typedef struct {
    float torque_nm;
    float unconstrained_torque_nm;
    float feedforward_nm;
    float feedback_nm;
    float integral_nm;
    float compensation_nm;
    float disturbance_nm;
    float position_error_rad;        /* reference - measurement */
    float velocity_error_rad_s;      /* Error used for feedback, after optional filter. */
    uint32_t flags;
    YawStatus status;
} YawOutput;

/* Exposed for static allocation and diagnostics, not for direct mutation. */
typedef struct {
    YawConfig config;
    float observer_position_rad;
    float observer_velocity_rad_s;
    float observer_disturbance_nm;
    float integral_nm;
    float compensation_nm;
    float last_torque_nm;
    float last_velocity_rad_s;
    float velocity_error_filtered_rad_s;
    bool initialized;
    bool ready;
    YawStatus fault;
} YawController;

bool yaw_config_valid(const YawConfig *config);
bool yaw_controller_init(YawController *controller, const YawConfig *config);
/* Explicitly clears a latched fault/history; next valid step outputs zero. */
void yaw_controller_reset(YawController *controller);
/* A lower limit takes precedence over slew continuity on the next step. */
bool yaw_controller_set_torque_limit(YawController *controller, float limit_nm);
/* Faults latch and return exact software zero, bypassing slew. Caller must
 * implement motor disable/watchdog; zero torque alone is not a physical stop.
 * A successful call represents one measured control interval. Never silently
 * substitute nominal dt. First valid call seeds ESO and returns YAW_WARMUP. */
YawStatus yaw_controller_step(YawController *controller,
                              const YawFeedback *feedback,
                              const YawReference *reference,
                              float dt_s, YawOutput *output);
const char *yaw_status_string(YawStatus status);

#ifdef __cplusplus
}
#endif
#endif
