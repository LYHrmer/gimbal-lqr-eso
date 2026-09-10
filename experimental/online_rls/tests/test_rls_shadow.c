#include "rls_shadow.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned failures;
static unsigned checks;

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        ++failures; \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
    } \
} while (0)

static RlsShadowConfig default_config(uint32_t n)
{
    RlsShadowConfig config;
    uint32_t i;
    memset(&config, 0, sizeof(config));
    config.dimension = n;
    config.forgetting_factor = 0.999f;
    config.covariance_initial = 50.0f;
    config.covariance_max = 1000000.0f;
    for (i = 0U; i < RLS_SHADOW_MAX_DIM; ++i) {
        config.theta_min[i] = -10.0f;
        config.theta_max[i] = 10.0f;
    }
    return config;
}

static void check_rejection(RlsShadow *state, const float *phi, float y,
                            int sample_valid, int excitation_ok,
                            RlsShadowStatus expected)
{
    RlsShadow before = *state;
    CHECK(rls_shadow_update(state, phi, y, sample_valid, excitation_ok) == expected);
    CHECK(memcmp(before.theta, state->theta, sizeof(state->theta)) == 0);
    CHECK(memcmp(before.p, state->p, sizeof(state->p)) == 0);
    CHECK(memcmp(&before.config, &state->config, sizeof(state->config)) == 0);
    CHECK(state->accepted_count == before.accepted_count);
    CHECK(state->last_status == (uint32_t)expected);
    CHECK(state->rejected_count == (before.rejected_count == UINT32_MAX ?
                                    UINT32_MAX : before.rejected_count + 1U));
}

static void test_config_validation(void)
{
    const float bad_lambda[] = {0.0f, -0.1f, 1.0001f, NAN, INFINITY};
    RlsShadowConfig config;
    RlsShadow state;
    uint32_t i;
    for (i = 0U; i < sizeof(bad_lambda) / sizeof(bad_lambda[0]); ++i) {
        config = default_config(2U);
        config.forgetting_factor = bad_lambda[i];
        CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
        CHECK(state.initialized == 0U);
    }
    config = default_config(2U);
    config.forgetting_factor = 1.0f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    CHECK(rls_shadow_init(&state, &state.config) == RLS_SHADOW_OK);
    CHECK(rls_shadow_init(NULL, &config) == RLS_SHADOW_INVALID_ARGUMENT);
    CHECK(rls_shadow_init(&state, NULL) == RLS_SHADOW_INVALID_CONFIG);
    check_rejection(&state, NULL, 0.0f, 1, 1, RLS_SHADOW_NOT_INITIALIZED);
    CHECK(rls_shadow_update(NULL, NULL, 0.0f, 1, 1) == RLS_SHADOW_INVALID_ARGUMENT);

    config = default_config(0U);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(RLS_SHADOW_MAX_DIM + 1U);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(2U);
    config.covariance_initial = 0.0f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config.covariance_initial = NAN;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(2U);
    config.covariance_max = config.covariance_initial * 0.5f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config.covariance_max = INFINITY;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(2U);
    config.innovation_limit = -1.0f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config.innovation_limit = NAN;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(2U);
    config.theta_initial[0] = 11.0f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(2U);
    config.theta_min[0] = 1.0f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
    config = default_config(2U);
    config.theta_max[0] = NAN;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_INVALID_CONFIG);
}

static void test_transactional_rejection(void)
{
    RlsShadow state;
    RlsShadowConfig config = default_config(2U);
    float phi[2] = {0.5f, -0.5f};
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    CHECK(rls_shadow_update(&state, phi, 0.25f, 1, 1) == RLS_SHADOW_OK);
    check_rejection(&state, NULL, NAN, 0, 1, RLS_SHADOW_SAMPLE_INVALID);
    check_rejection(&state, NULL, NAN, 1, 0, RLS_SHADOW_EXCITATION_INSUFFICIENT);
    check_rejection(&state, NULL, 0.0f, 1, 1, RLS_SHADOW_INVALID_ARGUMENT);
    check_rejection(&state, phi, NAN, 1, 1, RLS_SHADOW_NONFINITE_INPUT);
    check_rejection(&state, phi, INFINITY, 1, 1, RLS_SHADOW_NONFINITE_INPUT);
    phi[0] = NAN;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NONFINITE_INPUT);
    phi[0] = INFINITY;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NONFINITE_INPUT);
    phi[0] = FLT_MAX;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NUMERICAL_REJECTED);

    config.innovation_limit = 0.1f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    phi[0] = 1.0f;
    phi[1] = 0.0f;
    check_rejection(&state, phi, 0.2f, 1, 1, RLS_SHADOW_INNOVATION_REJECTED);

    config.innovation_limit = 0.0f;
    config.theta_max[0] = 0.1f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    check_rejection(&state, phi, 1.0f, 1, 1, RLS_SHADOW_PARAMETER_BOUNDS);
    config.theta_max[0] = 10.0f;
    config.theta_min[0] = -0.1f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    check_rejection(&state, phi, -1.0f, 1, 1, RLS_SHADOW_PARAMETER_BOUNDS);

    config = default_config(2U);
    config.covariance_initial = 1.0f;
    config.covariance_max = 1.5f;
    config.forgetting_factor = 0.5f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_COVARIANCE_LIMIT);
    /* Even an unused zero regressor must not silently wind up P. */
    phi[0] = 0.0f;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_COVARIANCE_LIMIT);

    config = default_config(2U);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.p[0] = -1.0f;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NUMERICAL_REJECTED);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.p[1] = 1.0f;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NUMERICAL_REJECTED);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.p[0] = NAN;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NUMERICAL_REJECTED);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.p[0] = 2.0f * config.covariance_max;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_COVARIANCE_LIMIT);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.theta[0] = NAN;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_NUMERICAL_REJECTED);
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.config.forgetting_factor = NAN;
    check_rejection(&state, phi, 0.0f, 1, 1, RLS_SHADOW_INVALID_CONFIG);

    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    state.accepted_count = UINT32_MAX;
    state.rejected_count = UINT32_MAX;
    CHECK(rls_shadow_update(&state, phi, 0.0f, 1, 1) == RLS_SHADOW_OK);
    CHECK(state.accepted_count == UINT32_MAX);
    check_rejection(&state, phi, 0.0f, 0, 0, RLS_SHADOW_SAMPLE_INVALID);
}

static void test_scalar_weighted_batch_equivalence(void)
{
    RlsShadowConfig config = default_config(1U);
    RlsShadow state;
    double information;
    double rhs;
    uint32_t k;
    config.forgetting_factor = 0.98f;
    config.theta_initial[0] = -0.7f;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    information = 1.0 / (double)config.covariance_initial;
    rhs = information * (double)config.theta_initial[0];
    for (k = 0U; k < 400U; ++k) {
        float phi = cosf((float)k * 0.067f);
        float y = 1.7f * phi + 0.03f * sinf((float)k * 0.199f);
        information = (double)config.forgetting_factor * information +
                      (double)phi * (double)phi;
        rhs = (double)config.forgetting_factor * rhs + (double)phi * (double)y;
        CHECK(rls_shadow_update(&state, &phi, y, 1, 1) == RLS_SHADOW_OK);
        CHECK(fabs((double)state.theta[0] - rhs / information) < 0.00002);
        CHECK(fabs((double)state.p[0] - 1.0 / information) < 0.00002);
    }
}

static void test_twenty_thousand_updates(void)
{
    const float truth[RLS_SHADOW_MAX_DIM] = {1.8f, 0.27f, -0.31f, 0.15f, -0.4f, 0.09f};
    RlsShadowConfig config = default_config(RLS_SHADOW_MAX_DIM);
    RlsShadow state;
    uint32_t k;
    uint32_t i;
    uint32_t j;
    CHECK(rls_shadow_init(&state, &config) == RLS_SHADOW_OK);
    for (k = 0U; k < 20000U; ++k) {
        float phi[RLS_SHADOW_MAX_DIM];
        float y = 0.0001f * cosf(0.197f * (float)k);
        for (i = 0U; i < RLS_SHADOW_MAX_DIM; ++i) {
            phi[i] = sinf((0.019f + 0.027f * (float)i) * (float)k + 0.31f * (float)i);
            y += phi[i] * truth[i];
        }
        CHECK(rls_shadow_update(&state, phi, y, 1, 1) == RLS_SHADOW_OK);
        for (i = 0U; i < RLS_SHADOW_MAX_DIM; ++i) {
            CHECK(isfinite(state.theta[i]));
            for (j = 0U; j < RLS_SHADOW_MAX_DIM; ++j) {
                CHECK(isfinite(state.p[i * RLS_SHADOW_MAX_DIM + j]));
                CHECK(state.p[i * RLS_SHADOW_MAX_DIM + j] ==
                      state.p[j * RLS_SHADOW_MAX_DIM + i]);
            }
        }
    }
    CHECK(state.accepted_count == 20000U);
    CHECK(state.rejected_count == 0U);
    for (i = 0U; i < RLS_SHADOW_MAX_DIM; ++i) {
        CHECK(fabsf(state.theta[i] - truth[i]) < 0.002f);
        CHECK(state.p[i * RLS_SHADOW_MAX_DIM + i] > 0.0f);
    }
}

int main(void)
{
    test_config_validation();
    test_transactional_rejection();
    test_scalar_weighted_batch_equivalence();
    test_twenty_thousand_updates();
    printf("RLS shadow C tests: %u checks, %u failures; config=%zu bytes, state=%zu bytes\n",
           checks, failures, sizeof(RlsShadowConfig), sizeof(RlsShadow));
    return failures == 0U ? 0 : 1;
}
