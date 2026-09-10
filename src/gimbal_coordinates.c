#include "gimbal_coordinates.h"
#include <math.h>
#include <stddef.h>

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "Finite-value checks require disabling fast-math and finite-math-only"
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
