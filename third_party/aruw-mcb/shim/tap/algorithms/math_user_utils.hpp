// Host stand-in for taproot's math_user_utils.hpp: only what the vendored code uses.
#ifndef TAPROOT_MATH_USER_UTILS_HPP_
#define TAPROOT_MATH_USER_UTILS_HPP_

#include <cfloat>
#include <cmath>

#ifndef M_TWOPI
#define M_TWOPI (2 * M_PI)  // modm's
#endif
#define F32_MAX FLT_MAX

namespace tap::algorithms
{
inline bool compareFloatClose(float val1, float val2, float epsilon)
{
    return fabsf(val1 - val2) <= epsilon;
}

template <typename T>
inline T limitVal(T val, T min, T max)
{
    if (min > max) return val;
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
}  // namespace tap::algorithms

#endif  // TAPROOT_MATH_USER_UTILS_HPP_
