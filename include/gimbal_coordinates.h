#ifndef GIMBAL_COORDINATES_H
#define GIMBAL_COORDINATES_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Lift a wrapped target to the nearest turn of a continuous measured angle.
 * Radians throughout; an exact half-turn tie chooses the positive direction.
 * Use for orientation targets only. Explicit multi-turn motion references must
 * stay continuous and must not pass through this shortest-path conversion.
 * No encoder accumulation or kinematic allocation is performed here.
 * Long-running systems must manage float precision/rebase their coordinate
 * consistently; this helper cannot recover precision already lost in a float. */
bool gimbal_angle_near(float target_rad, float continuous_angle_rad, float *result_rad);

#ifdef __cplusplus
}
#endif
#endif
