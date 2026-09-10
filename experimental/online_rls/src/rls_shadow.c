#include "rls_shadow.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "Finite-value checks require disabling fast-math and finite-math-only"
#endif

#define AT(matrix, row, col) ((matrix)[(row) * RLS_SHADOW_MAX_DIM + (col)])

static int config_valid(const RlsShadowConfig *config)
{
    uint32_t i;
    if (config == NULL || config->dimension == 0U ||
        config->dimension > RLS_SHADOW_MAX_DIM ||
        !isfinite(config->forgetting_factor) ||
        config->forgetting_factor <= 0.0f || config->forgetting_factor > 1.0f ||
        !isfinite(config->covariance_initial) || config->covariance_initial <= 0.0f ||
        !isfinite(config->covariance_max) ||
        config->covariance_max < config->covariance_initial ||
        !isfinite(config->innovation_limit) || config->innovation_limit < 0.0f) {
        return 0;
    }
    for (i = 0U; i < config->dimension; ++i) {
        if (!isfinite(config->theta_initial[i]) ||
            !isfinite(config->theta_min[i]) || !isfinite(config->theta_max[i]) ||
            config->theta_min[i] > config->theta_initial[i] ||
            config->theta_initial[i] > config->theta_max[i]) {
            return 0;
        }
    }
    return 1;
}

static RlsShadowStatus covariance_valid(const float *p, uint32_t n, float limit)
{
    float lower[RLS_SHADOW_MAX_DIM * RLS_SHADOW_MAX_DIM] = {0.0f};
    uint32_t i;
    uint32_t j;
    uint32_t k;
    for (i = 0U; i < n; ++i) {
        for (j = 0U; j < n; ++j) {
            const float entry = AT(p, i, j);
            if (!isfinite(entry)) {
                return RLS_SHADOW_NUMERICAL_REJECTED;
            }
            if (fabsf(entry) > limit) {
                return RLS_SHADOW_COVARIANCE_LIMIT;
            }
            if (entry != AT(p, j, i)) {
                return RLS_SHADOW_NUMERICAL_REJECTED;
            }
        }
    }
    /* A small full Cholesky check rejects indefinite/roundoff-damaged P. */
    for (i = 0U; i < n; ++i) {
        for (j = 0U; j <= i; ++j) {
            float value = AT(p, i, j);
            for (k = 0U; k < j; ++k) {
                value -= AT(lower, i, k) * AT(lower, j, k);
            }
            if (!isfinite(value)) {
                return RLS_SHADOW_NUMERICAL_REJECTED;
            }
            if (i == j) {
                if (value <= 0.0f) {
                    return RLS_SHADOW_NUMERICAL_REJECTED;
                }
                AT(lower, i, j) = sqrtf(value);
            } else {
                AT(lower, i, j) = value / AT(lower, j, j);
                if (!isfinite(AT(lower, i, j))) {
                    return RLS_SHADOW_NUMERICAL_REJECTED;
                }
            }
        }
    }
    return RLS_SHADOW_OK;
}

static RlsShadowStatus reject(RlsShadow *state, RlsShadowStatus status)
{
    state->last_status = (uint32_t)status;
    if (state->rejected_count != UINT32_MAX) {
        ++state->rejected_count;
    }
    return status;
}

RlsShadowStatus rls_shadow_init(RlsShadow *state, const RlsShadowConfig *config)
{
    RlsShadowConfig copy;
    uint32_t i;
    if (state == NULL) {
        return RLS_SHADOW_INVALID_ARGUMENT;
    }
    if (!config_valid(config)) {
        memset(state, 0, sizeof(*state));
        state->last_status = (uint32_t)RLS_SHADOW_INVALID_CONFIG;
        return RLS_SHADOW_INVALID_CONFIG;
    }
    copy = *config; /* Also permits init(state, &state->config). */
    memset(state, 0, sizeof(*state));
    state->config = copy;
    for (i = 0U; i < copy.dimension; ++i) {
        state->theta[i] = copy.theta_initial[i];
        AT(state->p, i, i) = copy.covariance_initial;
    }
    state->initialized = 1U;
    state->last_status = (uint32_t)RLS_SHADOW_OK;
    return RLS_SHADOW_OK;
}

RlsShadowStatus rls_shadow_update(RlsShadow *state, const float *phi, float y,
                                  int sample_valid, int excitation_ok)
{
    float prior[RLS_SHADOW_MAX_DIM * RLS_SHADOW_MAX_DIM] = {0.0f};
    float a[RLS_SHADOW_MAX_DIM * RLS_SHADOW_MAX_DIM] = {0.0f};
    float ap[RLS_SHADOW_MAX_DIM * RLS_SHADOW_MAX_DIM] = {0.0f};
    float next_p[RLS_SHADOW_MAX_DIM * RLS_SHADOW_MAX_DIM] = {0.0f};
    float next_theta[RLS_SHADOW_MAX_DIM] = {0.0f};
    float gain[RLS_SHADOW_MAX_DIM] = {0.0f};
    float predicted_y = 0.0f;
    float denominator = 1.0f;
    float innovation;
    uint32_t n;
    uint32_t i;
    uint32_t j;
    uint32_t k;
    RlsShadowStatus status;

    if (state == NULL) {
        return RLS_SHADOW_INVALID_ARGUMENT;
    }
    if (state->initialized != 1U) {
        return reject(state, RLS_SHADOW_NOT_INITIALIZED);
    }
    if (!config_valid(&state->config)) {
        return reject(state, RLS_SHADOW_INVALID_CONFIG);
    }
    if (!sample_valid) {
        return reject(state, RLS_SHADOW_SAMPLE_INVALID);
    }
    if (!excitation_ok) {
        return reject(state, RLS_SHADOW_EXCITATION_INSUFFICIENT);
    }
    if (phi == NULL) {
        return reject(state, RLS_SHADOW_INVALID_ARGUMENT);
    }
    n = state->config.dimension;
    if (!isfinite(y)) {
        return reject(state, RLS_SHADOW_NONFINITE_INPUT);
    }
    for (i = 0U; i < n; ++i) {
        if (!isfinite(phi[i])) {
            return reject(state, RLS_SHADOW_NONFINITE_INPUT);
        }
        if (!isfinite(state->theta[i]) ||
            state->theta[i] < state->config.theta_min[i] ||
            state->theta[i] > state->config.theta_max[i]) {
            return reject(state, RLS_SHADOW_NUMERICAL_REJECTED);
        }
        predicted_y += phi[i] * state->theta[i];
    }
    innovation = y - predicted_y;
    if (!isfinite(innovation)) {
        return reject(state, RLS_SHADOW_NUMERICAL_REJECTED);
    }
    if (state->config.innovation_limit > 0.0f &&
        fabsf(innovation) > state->config.innovation_limit) {
        return reject(state, RLS_SHADOW_INNOVATION_REJECTED);
    }
    status = covariance_valid(state->p, n, state->config.covariance_max);
    if (status != RLS_SHADOW_OK) {
        return reject(state, status);
    }
    for (i = 0U; i < n; ++i) {
        for (j = 0U; j < n; ++j) {
            AT(prior, i, j) = AT(state->p, i, j) / state->config.forgetting_factor;
            if (!isfinite(AT(prior, i, j))) {
                return reject(state, RLS_SHADOW_NUMERICAL_REJECTED);
            }
            gain[i] += AT(prior, i, j) * phi[j];
        }
        denominator += phi[i] * gain[i];
    }
    if (!isfinite(denominator) || denominator <= 0.0f) {
        return reject(state, RLS_SHADOW_NUMERICAL_REJECTED);
    }
    for (i = 0U; i < n; ++i) {
        gain[i] /= denominator;
        next_theta[i] = state->theta[i] + gain[i] * innovation;
        if (!isfinite(gain[i]) || !isfinite(next_theta[i])) {
            return reject(state, RLS_SHADOW_NUMERICAL_REJECTED);
        }
        if (next_theta[i] < state->config.theta_min[i] ||
            next_theta[i] > state->config.theta_max[i]) {
            return reject(state, RLS_SHADOW_PARAMETER_BOUNDS);
        }
        for (j = 0U; j < n; ++j) {
            AT(a, i, j) = (i == j ? 1.0f : 0.0f) - gain[i] * phi[j];
        }
    }
    /* Joseph update P+ = (I - K phi') Pprior (I - K phi')' + K K', R=1. */
    for (i = 0U; i < n; ++i) {
        for (j = 0U; j < n; ++j) {
            for (k = 0U; k < n; ++k) {
                AT(ap, i, j) += AT(a, i, k) * AT(prior, k, j);
            }
        }
    }
    for (i = 0U; i < n; ++i) {
        for (j = 0U; j < n; ++j) {
            float entry = gain[i] * gain[j];
            for (k = 0U; k < n; ++k) {
                entry += AT(ap, i, k) * AT(a, j, k);
            }
            AT(next_p, i, j) = entry;
        }
    }
    for (i = 0U; i < n; ++i) {
        for (j = 0U; j < i; ++j) {
            const float symmetric = 0.5f * AT(next_p, i, j) +
                                    0.5f * AT(next_p, j, i);
            AT(next_p, i, j) = symmetric;
            AT(next_p, j, i) = symmetric;
        }
    }
    status = covariance_valid(next_p, n, state->config.covariance_max);
    if (status != RLS_SHADOW_OK) {
        return reject(state, status);
    }

    /* Transaction commits only after ALL parameter and matrix checks pass. */
    memcpy(state->theta, next_theta, sizeof(state->theta));
    memcpy(state->p, next_p, sizeof(state->p));
    if (state->accepted_count != UINT32_MAX) {
        ++state->accepted_count;
    }
    state->last_status = (uint32_t)RLS_SHADOW_OK;
    return RLS_SHADOW_OK;
}
