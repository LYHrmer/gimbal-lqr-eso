#include "gimbal_coordinates.h"
#include <math.h>
#include <stddef.h>

#ifdef __FAST_MATH__
#error "gimbal_coordinates requires finite-value checks; compile without -ffast-math"
#endif

bool gimbal_angle_near(float target, float current, float *result)
{
    if (result == NULL) return false;
    *result = 0.0f;
    if (!isfinite(target) || !isfinite(current)) return false;
    const double pi = 3.14159265358979323846;
    double delta = remainder((double)target - (double)current, 2.0 * pi);
    if (delta <= -pi) delta += 2.0 * pi;
    const float near = (float)((double)current + delta);
    if (!isfinite(near)) return false;
    *result = near;
    return true;
}
