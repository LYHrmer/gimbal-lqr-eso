#include "gm6020.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static gm6020_config_t fixture(void)
{
    /* Offline protocol fixture; current_mode_confirmed is not a drive command. */
    const gm6020_config_t config = {0.741, 3.0, 3.0 * 0.741, true};
    return config;
}

static void test_torque_vectors_and_quantization(void)
{
    gm6020_config_t config = fixture();
    gm6020_current_word_t word = {0};
    const double lsb_nm = 3.0 / 16384.0 * config.torque_constant_nm_a;
    int i;
    assert(gm6020_torque_to_word(&config, 0.0, &word) == GM6020_OK);
    assert(word.valid && word.raw == 0 && word.current_wire_a == 0.0);
    assert(word.torque_wire_nm == 0.0 && !word.limited);
    assert(gm6020_torque_to_word(&config, config.torque_limit_nm, &word) == GM6020_OK);
    assert(word.raw == 16384 && word.current_wire_a == 3.0);
    assert(gm6020_torque_to_word(&config, -config.torque_limit_nm, &word) == GM6020_OK);
    assert(word.raw == -16384 && word.current_wire_a == -3.0);
    assert(gm6020_torque_to_word(&config, 100.0, &word) == GM6020_OK);
    assert(word.limited && word.raw == 16384);
    assert(gm6020_torque_to_word(&config, -100.0, &word) == GM6020_OK);
    assert(word.limited && word.raw == -16384);
    for (i = -10000; i <= 10000; ++i) {
        const double request = (double)i / 10000.0 * config.torque_limit_nm;
        assert(gm6020_torque_to_word(&config, request, &word) == GM6020_OK);
        assert(fabs(word.torque_wire_nm - request) <= 0.5 * lsb_nm + 1e-12);
        assert(fabs(word.current_wire_a) <= 3.0);
        assert(fabs(word.torque_wire_nm) <= config.torque_limit_nm);
    }
    config.current_limit_a = 0.5;
    assert(gm6020_torque_to_word(&config, 2.0, &word) == GM6020_OK);
    assert(word.limited && word.current_wire_a <= 0.5);
    assert(word.raw == 2730); /* Nearest 2731 would exceed 0.5 A. */
    config = fixture(); config.torque_limit_nm = 0.1;
    assert(gm6020_torque_to_word(&config, 1.0, &word) == GM6020_OK);
    assert(word.limited && word.torque_wire_nm <= 0.1);
    assert(0.1 - word.torque_wire_nm < lsb_nm);
    config.torque_limit_nm = 1e-10;
    assert(gm6020_torque_to_word(&config, 1.0, &word) == GM6020_OK);
    assert(word.raw == 0 && word.torque_wire_nm == 0.0);
}

static void test_mode_and_invalid_conversion(void)
{
    gm6020_config_t config = fixture();
    gm6020_current_word_t word = {0};
    const double invalid[] = {NAN, INFINITY, -INFINITY};
    size_t i;
    assert(gm6020_torque_to_word(&config, 0.5, &word) == GM6020_OK);
    const int16_t prior = word.raw;
    config.current_mode_confirmed = false;
    assert(gm6020_torque_to_word(&config, 0.0, &word) == GM6020_ERR_MODE_UNCONFIRMED);
    assert(!word.valid && isnan(word.current_wire_a) && isnan(word.torque_wire_nm));
    assert(word.raw == prior);
    config.current_mode_confirmed = true;
    for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        assert(gm6020_torque_to_word(&config, invalid[i], &word) == GM6020_ERR_NONFINITE);
        assert(!word.valid && word.raw == prior);
        config = fixture(); config.torque_constant_nm_a = invalid[i];
        assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
        config = fixture(); config.current_limit_a = invalid[i];
        assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
        config = fixture(); config.torque_limit_nm = invalid[i];
        assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
        config = fixture();
    }
    config.current_limit_a = 3.01;
    assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
    config.current_limit_a = 0.0;
    assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
    config = fixture(); config.torque_limit_nm = 0.0;
    assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
    config = fixture(); config.torque_limit_nm = 3.0;
    assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
    config = fixture(); config.torque_constant_nm_a = -1.0;
    assert(gm6020_validate_config(&config) == GM6020_ERR_CONFIG);
    assert(gm6020_torque_to_word(NULL, 0.0, &word) == GM6020_ERR_ARGUMENT);
    assert(!word.valid && word.raw == prior);
    assert(gm6020_torque_to_word(&config, 0.0, NULL) == GM6020_ERR_ARGUMENT);
}

static void test_complete_group_frames(void)
{
    gm6020_command_t command = {0};
    int16_t slots[4] = {-16384, -1, 0, 16384};
    const uint8_t expected[8] = {0xc0, 0, 0xff, 0xff, 0, 0, 0x40, 0};
    uint8_t prior[8];
    assert(gm6020_pack_current_group(0x1feU, slots, 4U, true, &command) == GM6020_OK);
    assert(command.valid && command.dlc == 8U && command.can_id == 0x1feU);
    assert(memcmp(command.data, expected, sizeof(expected)) == 0);
    memcpy(prior, command.data, sizeof(prior));
    assert(gm6020_pack_current_group(0x2feU, slots, 4U, true, &command) ==
           GM6020_ERR_CURRENT_RANGE); /* ID8 slot cannot contain a command. */
    assert(!command.valid && command.dlc == 0U && command.can_id == UINT32_MAX);
    assert(memcmp(command.data, prior, sizeof(prior)) == 0);
    assert(gm6020_pack_current_group(0x1feU, slots, 3U, true, &command) ==
           GM6020_ERR_ARGUMENT); /* A partial group is never padded. */
    assert(gm6020_pack_current_group(0x1feU, slots, 4U, false, &command) ==
           GM6020_ERR_MODE_UNCONFIRMED);
    assert(gm6020_pack_current_group(0x1ffU, slots, 4U, true, &command) ==
           GM6020_ERR_GROUP); /* No current-to-voltage fallback. */
    slots[3] = 0;
    assert(gm6020_pack_current_group(0x2feU, slots, 4U, true, &command) == GM6020_OK);
    assert(command.can_id == 0x2feU && command.data[6] == 0U && command.data[7] == 0U);
    slots[0] = 16385;
    assert(gm6020_pack_current_group(0x1feU, slots, 4U, true, &command) ==
           GM6020_ERR_CURRENT_RANGE);
    slots[0] = -16385;
    assert(gm6020_pack_current_group(0x1feU, slots, 4U, true, &command) ==
           GM6020_ERR_CURRENT_RANGE);
    assert(gm6020_pack_current_group(0x1feU, NULL, 4U, true, &command) ==
           GM6020_ERR_ARGUMENT);
    assert(gm6020_pack_current_group(0x1feU, slots, 4U, true, NULL) ==
           GM6020_ERR_ARGUMENT);
}

static void test_feedback(void)
{
    gm6020_feedback_t feedback = {0};
    uint8_t data[8] = {0x10, 0, 0xff, 0xc4, 0x80, 0, 55, 0xaa};
    size_t length;
    assert(gm6020_decode_feedback(1U, 0x205U, false, false, data, 8U, &feedback) == GM6020_OK);
    assert(feedback.valid && feedback.encoder_raw == 4096U && feedback.motor_id == 1U);
    assert(feedback.velocity_rpm == -60 && feedback.current_raw == -32768);
    assert(feedback.temperature_c == 55U);
    assert(fabs(feedback.position_rad - 3.141592653589793) < 1e-14);
    assert(fabs(feedback.velocity_rad_s + 6.283185307179586) < 1e-14);
    data[0] = 0x1f; data[1] = 0xff;
    data[2] = 0x7f; data[3] = 0xff;
    data[4] = 0x7f; data[5] = 0xff;
    assert(gm6020_decode_feedback(7U, 0x20bU, false, false, data, 8U, &feedback) == GM6020_OK);
    assert(feedback.encoder_raw == 8191U && feedback.position_rad < 6.283185307179586);
    assert(feedback.velocity_rpm == 32767 && feedback.current_raw == 32767);
    data[0] = 0x20; data[1] = 0;
    assert(gm6020_decode_feedback(7U, 0x20bU, false, false, data, 8U, &feedback) ==
           GM6020_ERR_ENCODER_RANGE);
    assert(!feedback.valid && isnan(feedback.position_rad) && isnan(feedback.velocity_rad_s));
    data[0] = 0;
    data[2] = 0x80; data[3] = 0;
    assert(gm6020_decode_feedback(1U, 0x205U, false, false, data, 8U, &feedback) == GM6020_OK);
    assert(feedback.encoder_raw == 0U && feedback.position_rad == 0.0);
    assert(feedback.velocity_rpm == -32768);
    for (length = 0U; length <= 9U; ++length) {
        if (length == 8U) continue;
        assert(gm6020_decode_feedback(1U, 0x205U, false, false, data, length, &feedback) ==
               GM6020_ERR_FRAME_FORMAT);
    }
    assert(gm6020_decode_feedback(1U, 0x205U, true, false, data, 8U, &feedback) ==
           GM6020_ERR_FRAME_FORMAT);
    assert(gm6020_decode_feedback(1U, 0x205U, false, true, data, 8U, &feedback) ==
           GM6020_ERR_FRAME_FORMAT);
    assert(gm6020_decode_feedback(1U, 0x805U, false, false, data, 8U, &feedback) ==
           GM6020_ERR_FRAME_FORMAT);
    assert(gm6020_decode_feedback(1U, 0x206U, false, false, data, 8U, &feedback) ==
           GM6020_ERR_MOTOR_ID);
    assert(gm6020_decode_feedback(0U, 0x204U, false, false, data, 8U, &feedback) ==
           GM6020_ERR_MOTOR_ID);
    assert(gm6020_decode_feedback(8U, 0x20cU, false, false, data, 8U, &feedback) ==
           GM6020_ERR_MOTOR_ID);
    assert(gm6020_decode_feedback(1U, 0x205U, false, false, NULL, 8U, &feedback) ==
           GM6020_ERR_ARGUMENT);
    assert(gm6020_decode_feedback(1U, 0x205U, false, false, data, 8U, NULL) ==
           GM6020_ERR_ARGUMENT);
}

int main(void)
{
    test_torque_vectors_and_quantization();
    test_mode_and_invalid_conversion();
    test_complete_group_frames();
    test_feedback();
    puts("gm6020: all conversion, group, mode and feedback tests passed");
    return 0;
}
