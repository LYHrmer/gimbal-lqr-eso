#include "dm_mit.h"

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "Finite-value checks require disabling fast-math and finite-math-only"
#endif

#include <math.h>
#include <string.h>

/* Normalize before multiplying, so even large finite ranges do not overflow. */
static double decode_bipolar(uint16_t raw, double maximum, uint32_t full_scale)
{
    const double normalized =
        (2.0 * (double)raw - (double)full_scale) / (double)full_scale;
    return normalized * maximum;
}

dm_mit_result_t dm_mit_validate_config(const dm_mit_config_t *config)
{
    double minimum_magnitude;
    if (config == NULL) {
        return DM_MIT_ERR_ARGUMENT;
    }
    if (!isfinite(config->p_max_rad) || config->p_max_rad <= 0.0 ||
        !isfinite(config->v_max_rad_s) || config->v_max_rad_s <= 0.0 ||
        !isfinite(config->t_max_nm) || config->t_max_nm <= 0.0 ||
        !isfinite(config->torque_limit_nm) ||
        config->torque_limit_nm <= 0.0 ||
        config->torque_limit_nm > config->t_max_nm ||
        config->motor_can_id == 0U || config->motor_can_id > 0x7ffU ||
        config->master_can_id > 0x7ffU) {
        return DM_MIT_ERR_CONFIG;
    }
    /* No exact zero exists in a bipolar 12-bit map with 4095 intervals. */
    minimum_magnitude = decode_bipolar(2048U, config->t_max_nm, 4095U);
    if (minimum_magnitude <= 0.0 ||
        config->torque_limit_nm < minimum_magnitude) {
        return DM_MIT_ERR_CONFIG;
    }
    return DM_MIT_OK;
}

dm_mit_result_t dm_mit_encode_torque(const dm_mit_config_t *config,
                                   double torque_nm,
                                   dm_mit_command_t *out)
{
    dm_mit_result_t result;
    uint16_t torque_raw;
    double torque_wire;
    double normalized;
    uint8_t payload[8];

    if (out == NULL) {
        return DM_MIT_ERR_ARGUMENT;
    }
    out->valid = false;
    out->dlc = 0U;
    out->can_id = UINT32_MAX;
    out->torque_wire_nm = NAN;
    result = dm_mit_validate_config(config);
    if (result != DM_MIT_OK) {
        return result;
    }
    if (!isfinite(torque_nm)) {
        return DM_MIT_ERR_NONFINITE;
    }
    if (fabs(torque_nm) > config->torque_limit_nm) {
        return DM_MIT_ERR_TORQUE_RANGE;
    }

    normalized = (torque_nm / config->t_max_nm + 1.0) * 2047.5;
    torque_raw = (uint16_t)floor(normalized + 0.5);
    torque_wire = decode_bipolar(torque_raw, config->t_max_nm, 4095U);
    if (fabs(torque_wire) > config->torque_limit_nm) {
        /* Nearest code can fall just outside the application limit. */
        torque_raw = (uint16_t)(torque_raw < 2048U ? torque_raw + 1U
                                                  : torque_raw - 1U);
        torque_wire = decode_bipolar(torque_raw, config->t_max_nm, 4095U);
    }
    if (!isfinite(torque_wire) ||
        fabs(torque_wire) > config->torque_limit_nm) {
        return DM_MIT_ERR_CONFIG;
    }

    /* p=32768, v=2048, kp=0, kd=0. Nominal p/v are irrelevant at zero gains. */
    payload[0] = 0x80U;
    payload[1] = 0x00U;
    payload[2] = 0x80U;
    payload[3] = 0x00U;
    payload[4] = 0x00U;
    payload[5] = 0x00U;
    payload[6] = (uint8_t)(torque_raw >> 8U);
    payload[7] = (uint8_t)(torque_raw & 0xffU);
    memcpy(out->data, payload, sizeof(payload));
    out->can_id = config->motor_can_id;
    out->dlc = 8U;
    out->torque_wire_nm = torque_wire;
    out->valid = true;
    return DM_MIT_OK;
}

dm_mit_result_t dm_mit_decode_feedback(const dm_mit_config_t *config,
                                     uint32_t can_id,
                                     bool is_extended,
                                     bool is_remote,
                                     const uint8_t *data,
                                     size_t length,
                                     dm_mit_feedback_t *out)
{
    dm_mit_result_t result;
    dm_mit_feedback_t decoded = {0};

    if (out == NULL) {
        return DM_MIT_ERR_ARGUMENT;
    }
    out->valid = false;
    out->position_rad = NAN;
    out->velocity_rad_s = NAN;
    out->torque_nm = NAN;
    if (data == NULL) {
        return DM_MIT_ERR_ARGUMENT;
    }
    result = dm_mit_validate_config(config);
    if (result != DM_MIT_OK) {
        return result;
    }
    if (is_extended || is_remote || can_id > 0x7ffU || length != 8U) {
        return DM_MIT_ERR_FRAME_FORMAT;
    }
    if (can_id != config->master_can_id) {
        return DM_MIT_ERR_MASTER_ID;
    }
    decoded.motor_id_low4 = (uint8_t)(data[0] & 0x0fU);
    if (decoded.motor_id_low4 != (config->motor_can_id & 0x0fU)) {
        return DM_MIT_ERR_MOTOR_ID;
    }
    decoded.status = (uint8_t)(data[0] >> 4U);
    decoded.position_raw = (uint16_t)(((uint16_t)data[1] << 8U) | data[2]);
    decoded.velocity_raw = (uint16_t)(((uint16_t)data[3] << 4U) |
                                      (data[4] >> 4U));
    decoded.torque_raw = (uint16_t)(((uint16_t)(data[4] & 0x0fU) << 8U) |
                                    data[5]);
    decoded.mos_temperature_c = data[6];
    decoded.rotor_temperature_c = data[7];
    decoded.position_rad = decode_bipolar(decoded.position_raw,
                                          config->p_max_rad, 65535U);
    decoded.velocity_rad_s = decode_bipolar(decoded.velocity_raw,
                                            config->v_max_rad_s, 4095U);
    decoded.torque_nm = decode_bipolar(decoded.torque_raw,
                                      config->t_max_nm, 4095U);
    decoded.valid = true;
    *out = decoded;
    return DM_MIT_OK;
}
