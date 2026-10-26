// Host stand-in for taproot's clock: simulated time, advanced by the caller.
#ifndef TAPROOT_CLOCK_HPP_
#define TAPROOT_CLOCK_HPP_

#include <cstdint>

namespace tap::arch::clock
{
inline uint32_t simTimeMicros = 0;

inline void setTimeMicroseconds(uint32_t t) { simTimeMicros = t; }
inline uint32_t getTimeMicroseconds() { return simTimeMicros; }
inline uint32_t getTimeMilliseconds() { return simTimeMicros / 1000; }
}  // namespace tap::arch::clock

#endif  // TAPROOT_CLOCK_HPP_
