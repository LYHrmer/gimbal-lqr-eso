#ifndef GM6020_H
#define GM6020_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GM6020_OK = 0,
    GM6020_ERR_ARGUMENT,
    GM6020_ERR_CONFIG,
    GM6020_ERR_MODE_UNCONFIRMED,
    GM6020_ERR_NONFINITE,
    GM6020_ERR_GROUP,
    GM6020_ERR_CURRENT_RANGE,
    GM6020_ERR_FRAME_FORMAT,
    GM6020_ERR_MOTOR_ID,
    GM6020_ERR_ENCODER_RANGE
} gm6020_result_t;

typedef struct {
    /* Explicit model/calibration value: v1.4 gives 0.741 Nm/A. */
    double torque_constant_nm_a;
    double current_limit_a; /* Independent application cap, 0 < cap <= 3 A. */
    double torque_limit_nm; /* Independent application cap, <= 3 A * Kt. */
    bool current_mode_confirmed; /* Set by host after verifying actual drive. */
} gm6020_config_t;

typedef struct {
    int16_t raw; /* Signed current command, -16384..16384. */
    bool valid;
    bool limited; /* Input was reduced by application current/torque limits. */
    double current_wire_a;
    double torque_wire_nm; /* Nominal from quantized command; not measured. */
} gm6020_current_word_t;

typedef struct {
    uint32_t can_id;
    uint8_t data[8];
    uint8_t dlc;
    bool valid;
} gm6020_command_t;

typedef struct {
    bool valid; /* Decoding success is not authorization to run. */
    uint8_t motor_id;
    uint16_t encoder_raw; /* 0..8191, single-turn. */
    int16_t velocity_rpm;
    int16_t current_raw; /* Feedback A/count is intentionally not assumed. */
    uint8_t temperature_c;
    double position_rad;
    double velocity_rad_s;
} gm6020_feedback_t;

gm6020_result_t gm6020_validate_config(const gm6020_config_t *config);

/* Pure conversion, no CAN traffic. Requires explicit current-mode confirmation.
 * Clamp to BOTH app limits, then quantize to nearest code (ties away from zero).
 * Move the result inward if needed so its nominal values still obey app limits.
 * On failure: valid=false, wire values=NAN, raw unchanged; do not reuse raw.
 * A zero request is exactly representable by this signed-current protocol.
 */
gm6020_result_t gm6020_torque_to_word(const gm6020_config_t *config,
                                    double torque_nm,
                                    gm6020_current_word_t *out);

/* Assemble a COMPLETE group snapshot, never a single-motor convenience frame.
 * group_can_id: 0x1FE for IDs 1..4, 0x2FE for IDs 5..7.
 * count MUST be 4 and every slot must be explicitly supplied in protocol counts.
 * Group 0x2FE slot[3] MUST be zero. Every slot must be within +/-16384.
 * Host must confirm current mode for all addressed existing motors and aggregate
 * fresh per-motor commands before this call. No automatic voltage fallback.
 * On failure: valid=false, dlc=0, can_id=UINT32_MAX, payload unchanged.
 */
gm6020_result_t gm6020_pack_current_group(uint32_t group_can_id,
                                        const int16_t *slots,
                                        size_t count,
                                        bool all_current_modes_confirmed,
                                        gm6020_command_t *out);

/* Bare standard CAN data payload only, exactly eight bytes; expected ID 1..7.
 * Verify CAN ID == 0x204 + expected_motor_id, encoder range, signed RPM/current.
 * Temperature and current_raw are data; host handles fault/watchdog policies.
 * On failure: valid=false and physical position/velocity become NAN.
 */
gm6020_result_t gm6020_decode_feedback(uint8_t expected_motor_id,
                                     uint32_t can_id,
                                     bool is_extended,
                                     bool is_remote,
                                     const uint8_t *data,
                                     size_t length,
                                     gm6020_feedback_t *out);

#ifdef __cplusplus
}
#endif

#endif
