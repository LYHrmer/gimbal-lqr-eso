#include "dm_mit.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static dm_mit_config_t fixture(void)
{
    /* Protocol test fixture only. These are NOT verified hardware settings. */
    const dm_mit_config_t config = {12.5, 30.0, 10.0, 0.5, 1U, 0x11U};
    return config;
}

static uint16_t command_torque_raw(const dm_mit_command_t *command)
{
    return (uint16_t)(((uint16_t)(command->data[6] & 0x0fU) << 8U) |
                      command->data[7]);
}

static void assert_invalid(const dm_mit_command_t *command)
{
    assert(!command->valid);
    assert(command->dlc == 0U);
    assert(command->can_id == UINT32_MAX);
    assert(isnan(command->torque_wire_nm));
}

static void test_known_vectors_and_endpoints(void)
{
    dm_mit_config_t config = fixture();
    dm_mit_command_t command = {0};
    const uint8_t zero_payload[8] = {0x80, 0x00, 0x80, 0, 0, 0, 0x08, 0x00};
    const uint8_t low_payload[8] = {0x80, 0x00, 0x80, 0, 0, 0, 0x00, 0x00};
    const uint8_t high_payload[8] = {0x80, 0x00, 0x80, 0, 0, 0, 0x0f, 0xff};

    assert(dm_mit_encode_torque(&config, 0.0, &command) == DM_MIT_OK);
    assert(command.valid && command.dlc == 8U && command.can_id == 1U);
    assert(memcmp(command.data, zero_payload, sizeof(zero_payload)) == 0);
    assert(command.torque_wire_nm > 0.0);
    assert(fabs(command.torque_wire_nm - 10.0 / 4095.0) < 1e-15);

    config.torque_limit_nm = config.t_max_nm;
    assert(dm_mit_encode_torque(&config, -10.0, &command) == DM_MIT_OK);
    assert(memcmp(command.data, low_payload, sizeof(low_payload)) == 0);
    assert(command.torque_wire_nm == -10.0);
    assert(dm_mit_encode_torque(&config, 10.0, &command) == DM_MIT_OK);
    assert(memcmp(command.data, high_payload, sizeof(high_payload)) == 0);
    assert(command.torque_wire_nm == 10.0);
}

static void test_quantization_and_limit(void)
{
    dm_mit_config_t config = fixture();
    dm_mit_command_t command = {0};
    uint16_t previous = 0U;
    const double lsb = 20.0 / 4095.0;
    int i;

    config.torque_limit_nm = 10.0;
    for (i = 0; i <= 20000; ++i) {
        const double requested = -10.0 + 20.0 * (double)i / 20000.0;
        assert(dm_mit_encode_torque(&config, requested, &command) == DM_MIT_OK);
        assert(fabs(command.torque_wire_nm - requested) <= lsb / 2.0 + 1e-12);
        assert(command_torque_raw(&command) >= previous);
        previous = command_torque_raw(&command);
        /* Verify both gain fields remain exactly zero across every code. */
        assert((command.data[3] & 0x0fU) == 0U && command.data[4] == 0U);
        assert(command.data[5] == 0U && (command.data[6] & 0xf0U) == 0U);
    }
    config.torque_limit_nm = 0.5;
    for (i = -1000; i <= 1000; ++i) {
        const double requested = (double)i / 2000.0;
        assert(dm_mit_encode_torque(&config, requested, &command) == DM_MIT_OK);
        assert(fabs(command.torque_wire_nm) <= config.torque_limit_nm);
        assert(fabs(command.torque_wire_nm - requested) <= lsb + 1e-12);
    }
    assert(dm_mit_encode_torque(&config, 0.5, &command) == DM_MIT_OK);
    assert(command_torque_raw(&command) == 2149U); /* 2150 would exceed 0.5 Nm. */
    assert(dm_mit_encode_torque(&config, -0.5, &command) == DM_MIT_OK);
    assert(command_torque_raw(&command) == 1946U);

    config.torque_limit_nm = (1.0 / 4095.0) * config.t_max_nm;
    assert(dm_mit_encode_torque(&config, 0.0, &command) == DM_MIT_OK);
    assert(dm_mit_encode_torque(&config, -config.torque_limit_nm, &command) ==
           DM_MIT_OK);
    assert(command_torque_raw(&command) == 2047U);
    config.torque_limit_nm *= 0.999;
    assert(dm_mit_encode_torque(&config, 0.0, &command) == DM_MIT_ERR_CONFIG);
    assert_invalid(&command);
}

static void test_failures(void)
{
    dm_mit_config_t config = fixture();
    dm_mit_command_t command = {0};
    uint8_t prior_payload[8];
    const double invalid_inputs[] = {NAN, INFINITY, -INFINITY};
    const double invalid_ranges[] = {NAN, INFINITY, -INFINITY, 0.0, -1.0};
    size_t i;

    assert(dm_mit_encode_torque(&config, 0.1, &command) == DM_MIT_OK);
    memcpy(prior_payload, command.data, sizeof(prior_payload));
    for (i = 0; i < sizeof(invalid_inputs) / sizeof(invalid_inputs[0]); ++i) {
        assert(dm_mit_encode_torque(&config, invalid_inputs[i], &command) ==
               DM_MIT_ERR_NONFINITE);
        assert_invalid(&command);
        assert(memcmp(prior_payload, command.data, sizeof(prior_payload)) == 0);
    }
    assert(dm_mit_encode_torque(&config, nextafter(0.5, 1.0), &command) ==
           DM_MIT_ERR_TORQUE_RANGE);
    assert_invalid(&command);
    assert(dm_mit_encode_torque(&config, -0.6, &command) == DM_MIT_ERR_TORQUE_RANGE);
    assert_invalid(&command);

    for (i = 0; i < sizeof(invalid_ranges) / sizeof(invalid_ranges[0]); ++i) {
        config = fixture(); config.p_max_rad = invalid_ranges[i];
        assert(dm_mit_encode_torque(&config, 0.0, &command) == DM_MIT_ERR_CONFIG);
        config = fixture(); config.v_max_rad_s = invalid_ranges[i];
        assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
        config = fixture(); config.t_max_nm = invalid_ranges[i];
        assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
        config = fixture(); config.torque_limit_nm = invalid_ranges[i];
        assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
    }
    config = fixture(); config.torque_limit_nm = 11.0;
    assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
    config = fixture(); config.motor_can_id = 0x800U;
    assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
    config.motor_can_id = 0U;
    assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
    config = fixture(); config.master_can_id = 0x800U;
    assert(dm_mit_validate_config(&config) == DM_MIT_ERR_CONFIG);
    config = fixture(); config.master_can_id = 0U;
    assert(dm_mit_validate_config(&config) == DM_MIT_OK);
    config.p_max_rad = config.v_max_rad_s = config.t_max_nm = DBL_MAX;
    config.torque_limit_nm = DBL_MAX;
    assert(dm_mit_encode_torque(&config, DBL_MAX, &command) == DM_MIT_OK);
    assert(command.torque_wire_nm == DBL_MAX);
    assert(dm_mit_encode_torque(NULL, 0.0, &command) == DM_MIT_ERR_ARGUMENT);
    assert_invalid(&command);
    assert(dm_mit_encode_torque(&config, 0.0, NULL) == DM_MIT_ERR_ARGUMENT);
    assert(dm_mit_validate_config(NULL) == DM_MIT_ERR_ARGUMENT);
}

static void test_feedback(void)
{
    dm_mit_config_t config = fixture();
    dm_mit_feedback_t feedback = {0};
    uint8_t data[8] = {0xb1, 0x12, 0x34, 0x56, 0x78, 0x9a, 71, 82};
    unsigned status;
    size_t length;

    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                  &feedback) == DM_MIT_OK);
    assert(feedback.valid && feedback.status == 0xbU);
    assert(feedback.motor_id_low4 == 1U);
    assert(feedback.position_raw == 0x1234U);
    assert(feedback.velocity_raw == 0x567U && feedback.torque_raw == 0x89aU);
    assert(feedback.mos_temperature_c == 71U && feedback.rotor_temperature_c == 82U);
    assert(fabs(feedback.position_rad - (-12.5 + 25.0 * 0x1234 / 65535.0)) < 1e-12);
    assert(fabs(feedback.velocity_rad_s - (-30.0 + 60.0 * 0x567 / 4095.0)) < 1e-12);
    assert(fabs(feedback.torque_nm - (-10.0 + 20.0 * 0x89a / 4095.0)) < 1e-12);

    for (status = 0U; status <= 15U; ++status) {
        data[0] = (uint8_t)((status << 4U) | 1U);
        assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                      &feedback) == DM_MIT_OK);
        assert(feedback.status == status); /* Including unknown status values. */
    }
    memset(data + 1, 0, 7U);
    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                  &feedback) == DM_MIT_OK);
    assert(feedback.position_rad == -12.5 && feedback.velocity_rad_s == -30.0);
    assert(feedback.torque_nm == -10.0);
    memset(data + 1, 0xff, 7U);
    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                  &feedback) == DM_MIT_OK);
    assert(feedback.position_rad == 12.5 && feedback.velocity_rad_s == 30.0);
    assert(feedback.torque_nm == 10.0 && feedback.mos_temperature_c == 255U);
    /* Receiving torque beyond the application limit must remain observable. */
    assert(feedback.torque_nm > config.torque_limit_nm);

    for (length = 0U; length <= 9U; ++length) {
        if (length == 8U) continue;
        assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, length,
                                      &feedback) == DM_MIT_ERR_FRAME_FORMAT);
        assert(!feedback.valid && isnan(feedback.torque_nm));
    }
    assert(dm_mit_decode_feedback(&config, 0x11U, true, false, data, 8U,
                                  &feedback) == DM_MIT_ERR_FRAME_FORMAT);
    assert(dm_mit_decode_feedback(&config, 0x11U, false, true, data, 8U,
                                  &feedback) == DM_MIT_ERR_FRAME_FORMAT);
    assert(dm_mit_decode_feedback(&config, 0x811U, false, false, data, 8U,
                                  &feedback) == DM_MIT_ERR_FRAME_FORMAT);
    assert(dm_mit_decode_feedback(&config, 1U, false, false, data, 8U,
                                  &feedback) == DM_MIT_ERR_MASTER_ID);
    data[0] = 0xb2;
    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                  &feedback) == DM_MIT_ERR_MOTOR_ID);
    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, NULL, 8U,
                                  &feedback) == DM_MIT_ERR_ARGUMENT);
    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                  NULL) == DM_MIT_ERR_ARGUMENT);
    assert(dm_mit_decode_feedback(NULL, 0x11U, false, false, data, 8U,
                                  &feedback) == DM_MIT_ERR_ARGUMENT);
    config.motor_can_id = 0x121U; data[0] = 0xa1U;
    assert(dm_mit_decode_feedback(&config, 0x11U, false, false, data, 8U,
                                  &feedback) == DM_MIT_OK);
    assert(feedback.motor_id_low4 == 1U && feedback.status == 0xaU);
}

int main(void)
{
    test_known_vectors_and_endpoints();
    test_quantization_and_limit();
    test_failures();
    test_feedback();
    puts("dm_mit: all protocol, quantization, limit and invalid-frame tests passed");
    return 0;
}
