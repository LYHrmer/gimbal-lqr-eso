#include "gm6020.h"

#include <float.h>
#include <math.h>
#include <string.h>

#ifdef __FAST_MATH__
#error "GM6020 finite-value checks require compilation without fast-math"
#endif

static const double pi_value = 3.14159265358979323846264338327950288;

gm6020_result_t gm6020_validate_config(const gm6020_config_t *config)
{
    if (config == NULL) return GM6020_ERR_ARGUMENT;
    if (!isfinite(config->torque_constant_nm_a) ||
        config->torque_constant_nm_a <= 0.0 ||
        config->torque_constant_nm_a > DBL_MAX / 3.0 ||
        !isfinite(config->current_limit_a) || config->current_limit_a <= 0.0 ||
        config->current_limit_a > 3.0 ||
        !isfinite(config->torque_limit_nm) || config->torque_limit_nm <= 0.0 ||
        config->torque_limit_nm > 3.0 * config->torque_constant_nm_a) {
        return GM6020_ERR_CONFIG;
    }
    return GM6020_OK;
}

gm6020_result_t gm6020_torque_to_word(const gm6020_config_t *config,
                                    double torque_nm,
                                    gm6020_current_word_t *out)
{
    gm6020_result_t result;
    double bounded_torque, requested_current, bounded_current;
    double current_wire, torque_wire;
    bool limited;
    int16_t raw;

    if (out == NULL) return GM6020_ERR_ARGUMENT;
    out->valid = false;
    out->limited = false;
    out->current_wire_a = NAN;
    out->torque_wire_nm = NAN;
    result = gm6020_validate_config(config);
    if (result != GM6020_OK) return result;
    if (!config->current_mode_confirmed) return GM6020_ERR_MODE_UNCONFIRMED;
    if (!isfinite(torque_nm)) return GM6020_ERR_NONFINITE;

    bounded_torque = fmin(fabs(torque_nm), config->torque_limit_nm);
    requested_current = bounded_torque / config->torque_constant_nm_a;
    bounded_current = fmin(requested_current, config->current_limit_a);
    limited = bounded_torque < fabs(torque_nm) || bounded_current < requested_current;
    raw = (int16_t)lround(bounded_current * (16384.0 / 3.0));
    current_wire = (double)raw * (3.0 / 16384.0);
    torque_wire = current_wire * config->torque_constant_nm_a;
    if (current_wire > config->current_limit_a || torque_wire > config->torque_limit_nm) {
        /* Rounding may cross an application cap by less than half a code. */
        if (raw > 0) --raw;
        current_wire = (double)raw * (3.0 / 16384.0);
        torque_wire = current_wire * config->torque_constant_nm_a;
        limited = true;
    }
    if (!isfinite(current_wire) || !isfinite(torque_wire) ||
        current_wire > config->current_limit_a || torque_wire > config->torque_limit_nm) {
        return GM6020_ERR_CONFIG;
    }
    if (torque_nm < 0.0) {
        raw = (int16_t)-raw;
        current_wire = -current_wire;
        torque_wire = -torque_wire;
    }
    out->raw = raw;
    out->current_wire_a = current_wire;
    out->torque_wire_nm = torque_wire;
    out->limited = limited;
    out->valid = true;
    return GM6020_OK;
}

gm6020_result_t gm6020_pack_current_group(uint32_t group_can_id,
                                        const int16_t *slots,
                                        size_t count,
                                        bool all_current_modes_confirmed,
                                        gm6020_command_t *out)
{
    uint8_t payload[8];
    size_t slot;
    if (out == NULL) return GM6020_ERR_ARGUMENT;
    out->valid = false;
    out->dlc = 0U;
    out->can_id = UINT32_MAX;
    if (slots == NULL || count != 4U) return GM6020_ERR_ARGUMENT;
    if (!all_current_modes_confirmed) return GM6020_ERR_MODE_UNCONFIRMED;
    if (group_can_id != 0x1feU && group_can_id != 0x2feU) return GM6020_ERR_GROUP;
    if (group_can_id == 0x2feU && slots[3] != 0) return GM6020_ERR_CURRENT_RANGE;
    for (slot = 0U; slot < 4U; ++slot) {
        uint16_t encoded;
        if (slots[slot] < -16384 || slots[slot] > 16384) return GM6020_ERR_CURRENT_RANGE;
        encoded = (uint16_t)slots[slot]; /* Defined modulo conversion for negatives. */
        payload[2U * slot] = (uint8_t)(encoded >> 8U);
        payload[2U * slot + 1U] = (uint8_t)(encoded & 0xffU);
    }
    memcpy(out->data, payload, sizeof(payload));
    out->can_id = group_can_id;
    out->dlc = 8U;
    out->valid = true;
    return GM6020_OK;
}

static uint16_t unsigned_word(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static int16_t signed_word(const uint8_t *data)
{
    const uint16_t raw = unsigned_word(data);
    const int32_t signed_value = raw >= 32768U ? (int32_t)raw - 65536 : (int32_t)raw;
    return (int16_t)signed_value;
}

gm6020_result_t gm6020_decode_feedback(uint8_t expected_motor_id,
                                     uint32_t can_id,
                                     bool is_extended,
                                     bool is_remote,
                                     const uint8_t *data,
                                     size_t length,
                                     gm6020_feedback_t *out)
{
    gm6020_feedback_t decoded = {0};
    if (out == NULL) return GM6020_ERR_ARGUMENT;
    out->valid = false;
    out->position_rad = NAN;
    out->velocity_rad_s = NAN;
    if (data == NULL) return GM6020_ERR_ARGUMENT;
    if (expected_motor_id < 1U || expected_motor_id > 7U) return GM6020_ERR_MOTOR_ID;
    if (is_extended || is_remote || can_id > 0x7ffU || length != 8U)
        return GM6020_ERR_FRAME_FORMAT;
    if (can_id != 0x204U + expected_motor_id) return GM6020_ERR_MOTOR_ID;
    decoded.encoder_raw = unsigned_word(data);
    if (decoded.encoder_raw > 8191U) return GM6020_ERR_ENCODER_RANGE;
    decoded.motor_id = expected_motor_id;
    decoded.velocity_rpm = signed_word(data + 2);
    decoded.current_raw = signed_word(data + 4);
    decoded.temperature_c = data[6];
    decoded.position_rad = (double)decoded.encoder_raw * (2.0 * pi_value / 8192.0);
    decoded.velocity_rad_s = (double)decoded.velocity_rpm * (2.0 * pi_value / 60.0);
    decoded.valid = true;
    *out = decoded;
    return GM6020_OK;
}
