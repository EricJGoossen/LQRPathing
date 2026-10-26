// Host stand-in for the chassis: records what the controller commands instead of driving motors.
// setDesiredOutput takes the chassis-frame translation in wheel RPM and the rotation, as the real
// one does; WHEEL_RADIUS is the sentry's (sentry_chassis_constants.hpp, without FUDGE_FACTOR).
#ifndef HOLONOMIC_CHASSIS_SUBSYSTEM_HPP_
#define HOLONOMIC_CHASSIS_SUBSYSTEM_HPP_

#include "tap/algorithms/transforms/transform.hpp"

namespace aruwsrc::control::chassis
{
static constexpr float WHEEL_RADIUS = 0.0762f;

class HolonomicChassisSubsystem
{
public:
    void setDesiredOutput(float x, float y, float r)
    {
        desiredX = x;
        desiredY = y;
        desiredR = r;
    }
    bool allMotorsOnline() const { return true; }
    float getDesiredRotation() const { return desiredR; }

    float desiredX = 0, desiredY = 0, desiredR = 0;
};
}  // namespace aruwsrc::control::chassis

#endif  // HOLONOMIC_CHASSIS_SUBSYSTEM_HPP_
