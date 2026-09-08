#ifndef DM_MIT_H
#define DM_MIT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pure protocol code: no transport, mode changes, enable or disable commands. */
typedef enum {
    DM_MIT_OK = 0,
    DM_MIT_ERR_ARGUMENT,
    DM_MIT_ERR_CONFIG,
    DM_MIT_ERR_NONFINITE,
    DM_MIT_ERR_TORQUE_RANGE,
    DM_MIT_ERR_FRAME_FORMAT,
    DM_MIT_ERR_MASTER_ID,
    DM_MIT_ERR_MOTOR_ID
} dm_mit_result_t;

typedef struct {
    /* Read these three mapping ranges from the actual drive configuration. */
    double p_max_rad;
    double v_max_rad_s;
    double t_max_nm;
    /* Independent experiment limit; must not exceed t_max_nm. */
    double torque_limit_nm;
    uint32_t motor_can_id;  /* Standard command ID: 1..0x7ff. */
    uint32_t master_can_id; /* Expected standard feedback ID: 0..0x7ff. */
} dm_mit_config_t;

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    bool valid;
    /* Nominal value decoded from the actual payload, NOT measured torque. */
    double torque_wire_nm;
} dm_mit_command_t;

typedef struct {
    bool valid; /* Decoded successfully; does NOT authorize motor operation. */
    uint8_t motor_id_low4;
    uint8_t status; /* Raw high nibble. Includes fault and non-fault states. */
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    double position_rad;
    double velocity_rad_s;
    double torque_nm;
} dm_mit_feedback_t;

dm_mit_result_t dm_mit_validate_config(const dm_mit_config_t *config);

/*
 * Encode kp=kd=0 and nominal p_des=v_des=0; torque is in Nm.
 * Reject nonfinite/out-of-limit requests, rather than silently saturating them.
 * Nearest quantization (ties upward), moved inward if needed to respect limit.
 * On any failure with non-NULL out: valid=false, dlc=0, can_id=UINT32_MAX,
 * torque_wire_nm=NAN. Payload bytes stay unchanged and MUST NOT be sent.
 * On success transmit only as a standard CAN data frame, after the upper-level
 * safety state machine separately authorizes operation. Buffers must not alias.
 */
dm_mit_result_t dm_mit_encode_torque(const dm_mit_config_t *config,
                                   double torque_nm,
                                   dm_mit_command_t *out);

/*
 * Input is a bare CAN data payload, NOT a USB-to-CAN adapter envelope.
 * Check standard/data frame, exactly 8 bytes, master ID and low-four motor ID.
 * Unknown status values are preserved, not accepted as permission to drive.
 * On failure out->valid=false and numeric physical values become NAN.
 * Upper layer must track receive timestamps, status policy, temperature limits,
 * angle unwrapping, stale feedback, disable handling and drive TIMEOUT.
 */
dm_mit_result_t dm_mit_decode_feedback(const dm_mit_config_t *config,
                                     uint32_t can_id,
                                     bool is_extended,
                                     bool is_remote,
                                     const uint8_t *data,
                                     size_t length,
                                     dm_mit_feedback_t *out);

#ifdef __cplusplus
}
#endif

#endif
