#ifndef RLS_SHADOW_H
#define RLS_SHADOW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RLS_SHADOW_MAX_DIM 6U

typedef enum {
    RLS_SHADOW_OK = 0,
    RLS_SHADOW_INVALID_CONFIG = 1,
    RLS_SHADOW_INVALID_ARGUMENT = 2,
    RLS_SHADOW_NOT_INITIALIZED = 3,
    RLS_SHADOW_SAMPLE_INVALID = 4,
    RLS_SHADOW_EXCITATION_INSUFFICIENT = 5,
    RLS_SHADOW_NONFINITE_INPUT = 6,
    RLS_SHADOW_INNOVATION_REJECTED = 7,
    RLS_SHADOW_PARAMETER_BOUNDS = 8,
    RLS_SHADOW_COVARIANCE_LIMIT = 9,
    RLS_SHADOW_NUMERICAL_REJECTED = 10
} RlsShadowStatus;

typedef struct {
    uint32_t dimension;
    float forgetting_factor;  /* 0 < lambda <= 1; per ACCEPTED sample. */
    float covariance_initial; /* Initial diagonal P, strictly positive. */
    float covariance_max;     /* Maximum absolute entry of P; not an eigenvalue. */
    float innovation_limit;   /* Absolute y - phi' theta limit; 0 disables it. */
    float theta_initial[RLS_SHADOW_MAX_DIM];
    float theta_min[RLS_SHADOW_MAX_DIM];
    float theta_max[RLS_SHADOW_MAX_DIM];
} RlsShadowConfig;

typedef struct {
    RlsShadowConfig config;
    float theta[RLS_SHADOW_MAX_DIM];
    /* Row-major, fixed row stride RLS_SHADOW_MAX_DIM, even if dimension < 6. */
    float p[RLS_SHADOW_MAX_DIM * RLS_SHADOW_MAX_DIM];
    uint32_t accepted_count;
    uint32_t rejected_count;
    uint32_t last_status;      /* A value from RlsShadowStatus. */
    uint32_t initialized;
} RlsShadow;

/*
 * Research-only float32 RLS estimator. No actuator/controller access.
 * The host supplies normalized phi and y for y = phi' theta + residual, and
 * handles physical units/scaling, timestamps, excitation and sensor validity.
 * P is an inverse weighted information matrix, NOT a calibrated uncertainty.
 * Scalar measurement weight R = 1 is used after host normalization.
 *
 * Structs are public for logging/ctypes; treat state and config as read-only
 * after init. Use one owner/task; calls on the same instance are not concurrent.
 * Reset/reconfiguration requires init and discards the previous estimate.
 * Do not compile with -ffast-math: rejection relies on finite checks.
 */
RlsShadowStatus rls_shadow_init(RlsShadow *state,
                                const RlsShadowConfig *config);

/*
 * phi contains at least config.dimension floats. An invalid sample or lack of
 * host-assessed excitation freezes BOTH theta and P, without forgetting.
 * Every rejected candidate also freezes theta/P: limits are never implemented
 * by clipping just the reported parameter. Diagnostic counters saturate.
 * Invalid-sample/excitation gates run before reading phi/y (phi may then be NULL).
 * This estimator never proves persistent excitation or closed-loop consistency.
 */
RlsShadowStatus rls_shadow_update(RlsShadow *state, const float *phi, float y,
                                  int sample_valid, int excitation_ok);

#ifdef __cplusplus
}
#endif

#endif
