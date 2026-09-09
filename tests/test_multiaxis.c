#include "yaw_controller.h"
#include "gimbal_coordinates.h"
#include "../variants/dm4310/simulation_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void close_to(float a, float b, float tolerance) { assert(fabsf(a-b) < tolerance); }

int main(void)
{
    const float pi = 3.14159265358979323846f;
    float lifted = 0.0f;
    for (int turn = -100; turn <= 100; ++turn) {
        const float center = (float)turn * 2.0f * pi;
        const float current = center + 179.0f*pi/180.0f;
        assert(gimbal_angle_near(-179.0f*pi/180.0f, current, &lifted));
        close_to(lifted-current, 2.0f*pi/180.0f, 1e-4f);
        assert(gimbal_angle_near(179.0f*pi/180.0f, center-179.0f*pi/180.0f, &lifted));
        close_to(lifted-(center-179.0f*pi/180.0f), -2.0f*pi/180.0f, 1e-4f);
    }
    assert(!gimbal_angle_near(NAN, 0.0f, &lifted) && lifted == 0.0f);
    assert(!gimbal_angle_near(0.0f, INFINITY, &lifted));
    assert(!gimbal_angle_near(0.0f, 0.0f, NULL));

    /* Three separately allocated JOINT loops. Targets and each link's gravity
     * torque are already allocated by the host; this is not a coupled plant. */
    YawController joints[3];
    YawConfig cfg = dm4310_simulation_config();
    cfg.eso_bandwidth_rad_s = cfg.eso_gain = 0.0f;
    cfg.torque_slew_nm_s = 100000.0f;
    cfg.position_min_rad = -1000.0f;
    cfg.position_max_rad = 1000.0f;
    const float positions[3] = {20.0f*pi, -0.2f, 0.3f};
    const float loads[3] = {0.0f, 0.2f, -0.1f};
    YawOutput out[3];
    for (int i = 0; i < 3; ++i) {
        assert(yaw_controller_init(&joints[i], &cfg));
        YawFeedback f = {.position_rad=positions[i], .valid=true};
        YawReference r = {.position_rad=positions[i], .valid=true};
        assert(yaw_controller_step_with_load(&joints[i], &f, &r, 0.001f,
                                             loads[i], loads[i], &out[i]) == YAW_WARMUP);
        assert(yaw_controller_step_with_load(&joints[i], &f, &r, 0.001f,
                                             loads[i], loads[i], &out[i]) == YAW_OK);
        close_to(out[i].torque_nm, loads[i], 1e-6f);
    }
    assert(yaw_controller_latch_fault(&joints[1], YAW_POSITION_LIMIT, &out[1]) == YAW_POSITION_LIMIT);
    assert(joints[0].fault == YAW_OK && joints[2].fault == YAW_OK);
    YawFeedback f = {.position_rad=positions[0], .valid=true};
    YawReference r = {.position_rad=positions[0]+0.01f, .valid=true};
    assert(yaw_controller_step(&joints[0], &f, &r, 0.001f, &out[0]) == YAW_OK);
    assert(out[0].torque_nm > 0.0f && out[0].torque_nm < 1.0f);
    close_to(joints[2].last_torque_nm, loads[2], 1e-6f);
    puts("multiaxis: wrapped orientation lifting and three independent joint instances passed");
    return 0;
}
