// Host stand-in; see vector.hpp. A planar world-to-chassis transform: the chassis's position and
// yaw in the world. apply() on a Vector rotates it from the world frame into the chassis frame,
// as taproot's (rotation^T * v).
#ifndef TAPROOT_TRANSFORM_HPP_
#define TAPROOT_TRANSFORM_HPP_

#include <cmath>

#include "position.hpp"
#include "vector.hpp"

namespace tap::algorithms::transforms
{
class Transform
{
public:
    Transform(float x, float y, float yaw) : translation(x, y, 0), yaw(yaw) {}

    void update(float x, float y, float yaw)
    {
        translation = Position(x, y, 0);
        this->yaw = yaw;
    }

    const Position& getTranslation() const { return translation; }

    Vector apply(const Vector& v) const
    {
        const float c = std::cos(yaw), s = std::sin(yaw);
        return Vector(c * v.x() + s * v.y(), -s * v.x() + c * v.y(), v.z());
    }

private:
    Position translation;
    float yaw;
};
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_TRANSFORM_HPP_
