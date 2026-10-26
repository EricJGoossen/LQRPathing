// Host stand-in; see vector.hpp.
#ifndef TAPROOT_POSITION_HPP_
#define TAPROOT_POSITION_HPP_

#include "vector.hpp"

namespace tap::algorithms::transforms
{
class Position
{
public:
    Position() : x_(0), y_(0), z_(0) {}
    Position(float x, float y) : x_(x), y_(y), z_(0) {}
    Position(float x, float y, float z) : x_(x), y_(y), z_(z) {}

    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }

    Vector operator-(const Position& o) const { return Vector(x_ - o.x_, y_ - o.y_, z_ - o.z_); }
    Position operator+(const Vector& v) const { return Position(x_ + v.x(), y_ + v.y(), z_ + v.z()); }
    Position operator-(const Vector& v) const { return Position(x_ - v.x(), y_ - v.y(), z_ - v.z()); }

    static Position interpolate(const Position& a, const Position& b, const float t)
    {
        return a + (b - a) * t;
    }
    static float distance(const Position& a, const Position& b) { return (b - a).magnitude(); }

private:
    float x_, y_, z_;
};
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_POSITION_HPP_
