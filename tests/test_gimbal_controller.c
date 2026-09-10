#include "gimbal_controller.h"
#include "../variants/gm6020/pitch_simulation_config.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

static const float step_s = 0.001f;

static GimbalConfig config(void)
{
    return (GimbalConfig){
        .control = {
            .inertia_kg_m2 = 0.039f, .damping_nm_s_rad = 0.30f,
            .k_position = 10.0f, .k_velocity = 1.0f,
            .integral_limit_nm = 0.5f, .antiwindup_rate_s = 30.0f,
            .coulomb_velocity_rad_s = 0.05f,
            .eso_bandwidth_rad_s = 80.0f, .eso_gain = 1.0f,
            .disturbance_limit_nm = 2.0f, .compensation_limit_nm = 1.0f,
            .compensation_slew_nm_s = 100.0f, .torque_limit_nm = 3.0f,
            .torque_slew_nm_s = 10000.0f,
            .dt_min_s = 0.0005f, .dt_max_s = 0.003f,
            .feedback_timeout_s = 0.005f, .reference_timeout_s = 0.05f,
            .position_min_rad = -10.0f, .position_max_rad = 10.0f,
            .velocity_limit_rad_s = 30.0f, .tracking_error_limit_rad = 3.0f
        },
        .gravity_cos_nm = 0.7f, .gravity_sin_nm = 0.2f,
        .joint_min_rad = -1.0f, .joint_max_rad = 1.0f,
        .joint_margin_rad = 0.1f, .pitch_enabled = true
    };
}

static YawFeedback feedback(void)
{
    return (YawFeedback){.valid = true};
}

static YawReference reference(void)
{
    return (YawReference){.valid = true};
}

static GimbalPose pose(void)
{
    return (GimbalPose){.valid = true};
}

static void disable_observer(GimbalConfig *cfg)
{
    cfg->control.eso_bandwidth_rad_s = 0.0f;
    cfg->control.eso_gain = 0.0f;
}

static void test_config_validation(void)
{
    GimbalConfig cfg = config();
    GimbalController c;
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_init(&c, &c.config));
#define INVALID_CONFIG(field, value) do { \
    cfg = config(); cfg.field = (value); \
    assert(!gimbal_controller_init(&c, &cfg)); \
} while (0)
    INVALID_CONFIG(gravity_cos_nm, NAN);
    INVALID_CONFIG(gravity_sin_nm, INFINITY);
    INVALID_CONFIG(joint_min_rad, NAN);
    INVALID_CONFIG(joint_max_rad, -1.0f);
    INVALID_CONFIG(joint_margin_rad, -0.01f);
    INVALID_CONFIG(joint_margin_rad, 1.0f);
    INVALID_CONFIG(control.inertia_kg_m2, 0.0f);
#undef INVALID_CONFIG
    cfg = config();
    cfg.gravity_cos_nm = -0.7f; cfg.gravity_sin_nm = -0.2f;
    assert(gimbal_controller_init(&c, &cfg)); /* Gravity coefficients are signed. */
    cfg.pitch_enabled = false;
    assert(!gimbal_controller_init(&c, &cfg)); /* Do not silently discard a gravity model. */
    cfg.gravity_cos_nm = 0.0f; cfg.gravity_sin_nm = 0.0f;
    assert(gimbal_controller_init(&c, &cfg));
}

static void test_gravity_uses_its_own_angle(void)
{
    GimbalConfig cfg = config();
    disable_observer(&cfg);
    YawFeedback f = feedback(); f.position_rad = 0.7f;
    YawReference r = reference(); r.position_rad = f.position_rad;
    GimbalPose p = pose(); p.joint_position_rad = -0.2f;
    GimbalController c;
    GimbalOutput out;
    const float gravity_angles[] = {0.0f, 0.6f, -0.9f, 2.4f};
    for (unsigned i = 0; i < sizeof(gravity_angles) / sizeof(gravity_angles[0]); ++i) {
        p.gravity_angle_rad = gravity_angles[i];
        assert(gimbal_controller_init(&c, &cfg));
        assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_WARMUP);
        assert(out.control.torque_nm == 0.0f);
        assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);
        const float expected = cfg.gravity_cos_nm * cosf(p.gravity_angle_rad) +
            cfg.gravity_sin_nm * sinf(p.gravity_angle_rad);
        assert(fabsf(out.gravity_feedforward_nm - expected) < 1e-6f);
        assert(fabsf(out.control.torque_nm - expected) < 1e-6f);
        assert(fabsf(out.joint_reference_rad - p.joint_position_rad) < 1e-6f);
    }
    /* Changing the chassis/joint zero does not change a known world gravity
     * orientation. Using the encoder angle in cos() would fail this check. */
    const float before = out.control.torque_nm;
    p.joint_position_rad = 0.5f;
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);
    assert(fabsf(out.control.torque_nm - before) < 1e-6f);
}

static void test_known_load_previous_interval(void)
{
    GimbalConfig cfg = config();
    cfg.control.damping_nm_s_rad = 0.0f;
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    assert(yaw_controller_init(&c, &cfg.control));
    assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                        0.4f, 0.4f, &out) == YAW_WARMUP);
    /* A measured 0.4 N.m motor torque balanced a 0.4 N.m opposing load in
     * the preceding interval. The current load abruptly changes sign. */
    f.applied_torque_valid = true; f.applied_torque_nm = 0.4f;
    assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                        -0.3f, 0.4f, &out) == YAW_OK);
    assert(fabsf(out.disturbance_nm) < 1e-7f);
    assert(fabsf(out.torque_nm + 0.3f) < 1e-6f);

    /* Without torque telemetry, evolve the plant using the last executable
     * motor command. Each known load is held over one complete interval. */
    assert(yaw_controller_init(&c, &cfg.control));
    f = feedback();
    float previous_load = 0.4f;
    assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                        previous_load, previous_load, &out) == YAW_WARMUP);
    for (int i = 0; i < 300; ++i) {
        const float acceleration = (out.torque_nm - previous_load) /
            cfg.control.inertia_kg_m2;
        f.position_rad += step_s * f.velocity_rad_s +
            0.5f * step_s * step_s * acceleration;
        f.velocity_rad_s += step_s * acceleration;
        const float current_load = i % 2 == 0 ? -0.3f : 0.4f;
        assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                            current_load, previous_load, &out) == YAW_OK);
        assert(fabsf(out.disturbance_nm) < 2e-5f);
        previous_load = current_load;
    }
}

static void test_static_hold_without_double_compensation(void)
{
    /* This is a behavioral equilibrium test, not a performance benchmark.
     * World pitch, joint pitch and the gravity zero intentionally differ. */
    for (int trial = 0; trial < 4; ++trial) {
        const bool telemetry = trial % 2 != 0;
        const double residual_load = trial < 2 ? 0.0 : 0.12;
        GimbalConfig cfg = config();
        GimbalController c;
        GimbalOutput out;
        YawFeedback f = feedback();
        YawReference r = reference(); r.position_rad = 0.35f;
        GimbalPose p = pose();
        double position = 0.35, velocity = 0.0, previous_torque = 0.0;
        assert(gimbal_controller_init(&c, &cfg));
        for (int i = 0; i < 6000; ++i) {
            f.position_rad = (float)position;
            f.velocity_rad_s = (float)velocity;
            f.applied_torque_valid = telemetry;
            f.applied_torque_nm = (float)previous_torque;
            p.joint_position_rad = (float)(position - 0.30);
            p.joint_velocity_rad_s = (float)velocity;
            p.gravity_angle_rad = (float)(position + 0.25);
            const YawStatus status = gimbal_controller_step(&c, &f, &r, &p, step_s, &out);
            assert(status == (i == 0 ? YAW_WARMUP : YAW_OK));
            const double load = (double)cfg.gravity_cos_nm * cos(position + 0.25) +
                (double)cfg.gravity_sin_nm * sin(position + 0.25) + residual_load;
            const double acceleration = ((double)out.control.torque_nm - load -
                (double)cfg.control.damping_nm_s_rad * velocity) /
                (double)cfg.control.inertia_kg_m2;
            position += (double)step_s * velocity +
                0.5 * (double)step_s * (double)step_s * acceleration;
            velocity += (double)step_s * acceleration;
            previous_torque = (double)out.control.torque_nm;
        }
        assert(fabs(position - (double)r.position_rad) < 0.0005);
        assert(fabs(velocity) < 0.001);
        /* A separate unknown opposing load belongs in ESO; the modeled gravity
         * does not. This also checks the sign of residual compensation. */
        assert(fabs((double)out.control.disturbance_nm + residual_load) < 0.003);
        assert(fabs((double)out.control.compensation_nm - residual_load) < 0.003);
        assert(fabs((double)out.control.torque_nm -
                    (double)out.gravity_feedforward_nm - residual_load) < 0.003);
    }
}

static void test_total_torque_limits_and_antiwindup(void)
{
    GimbalConfig cfg = config();
    disable_observer(&cfg);
    cfg.gravity_cos_nm = 2.0f; cfg.gravity_sin_nm = 0.0f;
    cfg.control.torque_slew_nm_s = 20.0f;
    GimbalController c;
    GimbalOutput out;
    YawFeedback f = feedback();
    YawReference r = reference();
    GimbalPose p = pose();
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_WARMUP);
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);
    assert(out.control.torque_nm <= 0.020001f);
    assert((out.control.flags & YAW_SLEW_LIMITED) != 0u);
    for (int i = 0; i < 110; ++i)
        assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);
    assert(fabsf(out.control.torque_nm - 2.0f) < 1e-5f);
    assert(gimbal_controller_set_torque_limit(&c, 0.3f));
    assert(!gimbal_controller_set_torque_limit(&c, NAN));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);
    assert(fabsf(out.control.torque_nm) <= 0.3f);
    assert((out.control.flags & YAW_TORQUE_LIMITED) != 0u);
    assert(out.gravity_feedforward_nm == 2.0f);

    cfg.control.k_position = 1.0f; cfg.control.k_integral = 2.0f;
    cfg.control.integral_limit_nm = 2.0f; cfg.control.torque_limit_nm = 1.0f;
    cfg.control.torque_slew_nm_s = 10000.0f;
    r.position_rad = 0.2f;
    assert(gimbal_controller_init(&c, &cfg));
    for (int i = 0; i < 5000; ++i) {
        const YawStatus status = gimbal_controller_step(&c, &f, &r, &p, step_s, &out);
        assert(status == (i == 0 ? YAW_WARMUP : YAW_OK));
        assert(fabsf(out.control.torque_nm) <= 1.0f);
    }
    /* A stalled joint needs more gravity torque than available. Back-calculation
     * must see gravity as part of the saturated total, so the integral retreats. */
    assert(c.core.integral_nm < -1.0f && c.core.integral_nm > -1.3f);
}

static void test_pose_faults_latch_and_reset(void)
{
    GimbalConfig cfg = config();
    GimbalController c;
    GimbalOutput out;
    YawFeedback f = feedback();
    YawReference r = reference();
    GimbalPose p;
#define POSE_FAULT(change, expected) do { \
    p = pose(); \
    assert(gimbal_controller_init(&c, &cfg)); \
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_WARMUP); \
    change; \
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == (expected)); \
    assert(out.control.torque_nm == 0.0f); \
    p = pose(); \
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == (expected)); \
    gimbal_controller_reset(&c); \
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_WARMUP); \
    assert(out.control.torque_nm == 0.0f); \
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK); \
} while (0)
    POSE_FAULT(p.valid = false, YAW_BAD_FEEDBACK);
    POSE_FAULT(p.gravity_angle_rad = NAN, YAW_BAD_FEEDBACK);
    POSE_FAULT(p.joint_position_rad = INFINITY, YAW_BAD_FEEDBACK);
    POSE_FAULT(p.joint_velocity_rad_s = NAN, YAW_BAD_FEEDBACK);
    POSE_FAULT(p.age_s = NAN, YAW_BAD_FEEDBACK);
    POSE_FAULT(p.age_s = -0.001f, YAW_BAD_FEEDBACK);
    POSE_FAULT(p.age_s = 0.006f, YAW_STALE_FEEDBACK);
#undef POSE_FAULT
    /* Fresh world feedback never makes a missing mechanical snapshot usable. */
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, NULL, step_s, &out) == YAW_BAD_FEEDBACK);
    assert(out.control.torque_nm == 0.0f);
}

static void test_known_load_faults_preserve_first_reason(void)
{
    GimbalConfig cfg = config();
    YawController c;
    YawFeedback f = feedback();
    YawReference r = reference();
    YawOutput out;
    for (int previous = 0; previous < 2; ++previous) {
        assert(yaw_controller_init(&c, &cfg.control));
        assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                            0.1f, 0.1f, &out) == YAW_WARMUP);
        assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                            previous ? 0.1f : NAN,
                                            previous ? INFINITY : 0.1f,
                                            &out) == YAW_NUMERIC_FAULT);
        assert(out.torque_nm == 0.0f);
        assert(yaw_controller_latch_fault(&c, YAW_BAD_FEEDBACK, &out) == YAW_NUMERIC_FAULT);
        assert(out.torque_nm == 0.0f);
        assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                            0.1f, 0.1f, &out) == YAW_NUMERIC_FAULT);
        yaw_controller_reset(&c);
        assert(yaw_controller_step_with_load(&c, &f, &r, step_s,
                                            0.1f, 0.1f, &out) == YAW_WARMUP);
        assert(out.torque_nm == 0.0f);
    }
}

static void test_joint_limits_and_relative_velocity(void)
{
    GimbalConfig cfg = config();
    cfg.joint_margin_rad = 0.125f;
    disable_observer(&cfg);
    GimbalController c;
    GimbalOutput out;
    YawFeedback f = feedback(); f.position_rad = 0.5f;
    YawReference r = reference(); r.position_rad = 0.5f;
    GimbalPose p = pose();
    p.joint_position_rad = 1.01f;
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_POSITION_LIMIT);
    p.joint_position_rad = 0.4f;
    r.position_rad = 1.1f; /* World target is valid, inferred joint target 1.0 is not. */
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_POSITION_LIMIT);

    r.position_rad = f.position_rad;
    p.joint_position_rad = cfg.joint_max_rad - cfg.joint_margin_rad;
    p.joint_velocity_rad_s = 0.1f;
    f.velocity_rad_s = 0.5f; /* Chassis/world offset is moving at +0.4 rad/s. */
    r.velocity_rad_s = 0.5f; /* Joint target moves outward at +0.1 rad/s. */
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_POSITION_LIMIT);
    r.velocity_rad_s = 0.2f; /* Positive world speed, but inward joint target speed. */
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_WARMUP);
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);

    p.joint_position_rad = cfg.joint_min_rad + cfg.joint_margin_rad;
    p.joint_velocity_rad_s = -0.1f;
    f.velocity_rad_s = -0.5f;
    r.velocity_rad_s = -0.5f;
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_POSITION_LIMIT);
    r.velocity_rad_s = -0.2f;
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_WARMUP);
}

static void assert_inward_bounds(const GimbalConfig *cfg, float low, float high)
{
    const double exact_low = (double)cfg->joint_min_rad + (double)cfg->joint_margin_rad;
    const double exact_high = (double)cfg->joint_max_rad - (double)cfg->joint_margin_rad;
    assert(low < high);
    assert((double)low >= exact_low && (double)high <= exact_high);
    assert(low >= cfg->joint_min_rad && high <= cfg->joint_max_rad);
    /* The chosen endpoints are the closest representable values on the
     * permitted side, not arbitrary extra safety offsets. */
    assert((double)nextafterf(low, -INFINITY) < exact_low);
    assert((double)nextafterf(high, INFINITY) > exact_high);
}

static void test_representable_joint_reference_bounds(void)
{
    GimbalConfig cfg = gm6020_pitch_simulation_config();
    float low = 0.0f, high = 0.0f;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert_inward_bounds(&cfg, low, high);
    /* The profile's straightforward float lower clamp rounds outside the
     * requested one-degree inset. It must remain excluded. */
    const float naive_low = cfg.joint_min_rad + cfg.joint_margin_rad;
    assert(naive_low < low);
    assert(low == nextafterf(naive_low, INFINITY));
    assert(gimbal_config_valid(&cfg));

    cfg = config();
    cfg.joint_min_rad = -FLT_MAX;
    cfg.joint_max_rad = FLT_MAX;
    cfg.joint_margin_rad = FLT_MAX * 0.5f;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert_inward_bounds(&cfg, low, high);
    assert(gimbal_config_valid(&cfg));
    cfg.joint_margin_rad = FLT_MAX;
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(low == 0.0f && high == 0.0f && !gimbal_config_valid(&cfg));

    /* Do not use a double sum as the oracle here: it loses these positive
     * margins too. The mathematically correct nearest inward floats are known. */
    cfg = config();
    cfg.joint_margin_rad = FLT_TRUE_MIN;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(low == nextafterf(-1.0f, INFINITY));
    assert(high == nextafterf(1.0f, -INFINITY));
    cfg.joint_min_rad = 1e-30f;
    cfg.joint_max_rad = 3.0f;
    cfg.joint_margin_rad = 1.0f;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(low == nextafterf(1.0f, INFINITY) && high == 2.0f);
    cfg.joint_min_rad = -3.0f;
    cfg.joint_max_rad = -1e-30f;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(low == -2.0f && high == nextafterf(-1.0f, -INFINITY));
    cfg.joint_min_rad = -FLT_MAX;
    cfg.joint_max_rad = FLT_MAX;
    cfg.joint_margin_rad = 1.0f;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(low == nextafterf(-FLT_MAX, INFINITY));
    assert(high == nextafterf(FLT_MAX, -INFINITY));

    cfg = config();
    cfg.joint_min_rad = 1.0f;
    cfg.joint_max_rad = nextafterf(1.0f, INFINITY);
    const float ulp = cfg.joint_max_rad - cfg.joint_min_rad;
    cfg.joint_margin_rad = 0.0f;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert_inward_bounds(&cfg, low, high);
    assert(gimbal_config_valid(&cfg));
    cfg.joint_margin_rad = 0.25f * ulp; /* Positive double width, no float inside. */
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(!gimbal_config_valid(&cfg));
    cfg.joint_max_rad = nextafterf(cfg.joint_max_rad, INFINITY);
    cfg.joint_margin_rad = 0.75f * ulp; /* One float inside is still zero travel. */
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    assert(low == 0.0f && high == 0.0f && !gimbal_config_valid(&cfg));

    cfg = config();
    cfg.control.inertia_kg_m2 = NAN;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high)); /* Geometry only. */
    assert(!gimbal_config_valid(&cfg));
    cfg = config();
    cfg.pitch_enabled = false;
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    cfg = config(); cfg.joint_margin_rad = NAN;
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    cfg = config(); cfg.joint_min_rad = -INFINITY;
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    cfg = config(); cfg.joint_max_rad = INFINITY;
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    cfg = config(); cfg.joint_margin_rad = -0.1f;
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &high));
    cfg = config();
    assert(!gimbal_joint_reference_bounds(NULL, &low, &high));
    assert(low == 0.0f && high == 0.0f);
    assert(!gimbal_joint_reference_bounds(&cfg, NULL, &high) && high == 0.0f);
    assert(!gimbal_joint_reference_bounds(&cfg, &low, NULL) && low == 0.0f);
    assert(!gimbal_joint_reference_bounds(&cfg, &low, &low) && low == 0.0f);
}

static void test_representable_endpoints_and_outward_velocity(void)
{
    const GimbalConfig cfg = gm6020_pitch_simulation_config();
    float low, high;
    assert(gimbal_joint_reference_bounds(&cfg, &low, &high));
    YawFeedback f = feedback();
    YawReference r = reference();
    /* Keep the selected world reference equal to feedback: the encoder joint
     * offset, not the world angle, is what reaches a mechanical endpoint. */
    for (int side = 0; side < 2; ++side) {
        const float endpoint = side == 0 ? low : high;
        const float outward = side == 0 ? -0.1f : 0.1f;
        for (int direction = -1; direction <= 1; ++direction) {
            GimbalController c;
            GimbalOutput out;
            GimbalPose p = pose();
            p.joint_position_rad = endpoint;
            p.joint_velocity_rad_s = (float)direction * outward;
            assert(gimbal_controller_init(&c, &cfg));
            const YawStatus expected = direction > 0 ? YAW_POSITION_LIMIT : YAW_WARMUP;
            assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == expected);
            if (direction > 0) {
                assert(out.control.torque_nm == 0.0f);
                p.joint_velocity_rad_s = -outward;
                assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_POSITION_LIMIT);
            } else {
                assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_OK);
            }
        }
        for (int direction = -1; direction <= 1; direction += 2) {
            GimbalController c;
            GimbalOutput out;
            GimbalPose p = pose();
            const float toward = (float)direction * outward > 0.0f ? INFINITY : -INFINITY;
            p.joint_position_rad = nextafterf(endpoint, toward);
            assert(gimbal_controller_init(&c, &cfg));
            const YawStatus expected = direction > 0 ? YAW_POSITION_LIMIT : YAW_WARMUP;
            assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == expected);
        }
    }
    GimbalController c;
    GimbalOutput out;
    GimbalPose p = pose();
    p.joint_position_rad = cfg.joint_min_rad + cfg.joint_margin_rad;
    assert(gimbal_controller_init(&c, &cfg));
    assert(gimbal_controller_step(&c, &f, &r, &p, step_s, &out) == YAW_POSITION_LIMIT);
}

static void assert_same_control(const YawOutput *a, const YawOutput *b)
{
    assert(a->status == b->status && a->flags == b->flags);
    assert(a->torque_nm == b->torque_nm);
    assert(a->unconstrained_torque_nm == b->unconstrained_torque_nm);
    assert(a->disturbance_nm == b->disturbance_nm);
    assert(a->compensation_nm == b->compensation_nm);
    assert(a->integral_nm == b->integral_nm);
    assert(a->feedforward_nm == b->feedforward_nm);
    assert(a->feedback_nm == b->feedback_nm);
}

static void test_yaw_compatibility_and_instance_isolation(void)
{
    GimbalConfig cfg = config(); cfg.pitch_enabled = false;
    cfg.gravity_cos_nm = 0.0f; cfg.gravity_sin_nm = 0.0f;
    GimbalController yaw, pitch, independent_yaw;
    YawController original_yaw;
    GimbalOutput out_yaw, out_pitch, out_independent;
    YawOutput out_original;
    YawFeedback f = feedback();
    YawReference r = reference();
    GimbalPose p = pose();
    assert(gimbal_controller_init(&yaw, &cfg));
    assert(gimbal_controller_init(&independent_yaw, &cfg));
    assert(yaw_controller_init(&original_yaw, &cfg.control));
    cfg.pitch_enabled = true; cfg.gravity_cos_nm = 0.7f; cfg.gravity_sin_nm = 0.2f;
    assert(gimbal_controller_init(&pitch, &cfg));
    for (int i = 0; i < 300; ++i) {
        const float time = (float)i * step_s;
        f.position_rad = 0.1f * sinf(7.0f * time);
        f.velocity_rad_s = 0.7f * cosf(7.0f * time);
        r.position_rad = 0.15f * sinf(4.0f * time);
        r.velocity_rad_s = 0.6f * cosf(4.0f * time);
        r.acceleration_rad_s2 = -2.4f * sinf(4.0f * time);
        const YawStatus a = gimbal_controller_step(&yaw, &f, &r, NULL, step_s, &out_yaw);
        const YawStatus b = yaw_controller_step(&original_yaw, &f, &r, step_s, &out_original);
        assert(a == b);
        assert_same_control(&out_yaw.control, &out_original);
        assert(out_yaw.gravity_feedforward_nm == 0.0f);
        if (i == 150) p.valid = false;
        const YawStatus pitch_status = gimbal_controller_step(&pitch, &f, &r, &p, step_s, &out_pitch);
        assert(pitch_status == (i >= 150 ? YAW_BAD_FEEDBACK : (i == 0 ? YAW_WARMUP : YAW_OK)));
        if (i == 200) gimbal_controller_reset(&pitch);
        /* The other instance's observer, limiter and fault/reset state are private. */
        assert(gimbal_controller_step(&independent_yaw, &f, &r, NULL, step_s,
                                      &out_independent) == a);
        assert_same_control(&out_yaw.control, &out_independent.control);
    }
}

int main(void)
{
    test_config_validation();
    test_gravity_uses_its_own_angle();
    test_known_load_previous_interval();
    test_static_hold_without_double_compensation();
    test_total_torque_limits_and_antiwindup();
    test_pose_faults_latch_and_reset();
    test_known_load_faults_preserve_first_reason();
    test_joint_limits_and_relative_velocity();
    test_representable_joint_reference_bounds();
    test_representable_endpoints_and_outward_velocity();
    test_yaw_compatibility_and_instance_isolation();
    puts("gimbal_controller: 11 behavior test groups passed");
    return 0;
}
