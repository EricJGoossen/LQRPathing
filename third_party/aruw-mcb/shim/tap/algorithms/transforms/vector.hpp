// Host stand-in for taproot's transforms (Position, Vector, Transform), which are built on
// CMSIS-DSP matrices. Same interface for what the vendored code uses, plain floats underneath.
#ifndef TAPROOT_VECTOR_HPP_
#define TAPROOT_VECTOR_HPP_

#include <cmath>

namespace tap::algorithms::transforms
{
class Position;

class Vector
{
public:
    Vector(float x, float y, float z) : x_(x), y_(y), z_(z) {}

    float x() const { return x_; }
    float y() const { return y_; }
    float z() const { return z_; }

    Vector operator+(const Vector& o) const { return Vector(x_ + o.x_, y_ + o.y_, z_ + o.z_); }
    Vector operator-(const Vector& o) const { return Vector(x_ - o.x_, y_ - o.y_, z_ - o.z_); }
    Vector operator*(float s) const { return Vector(x_ * s, y_ * s, z_ * s); }
    Vector operator/(float s) const { return Vector(x_ / s, y_ / s, z_ / s); }

    float dot(const Vector& o) const { return x_ * o.x_ + y_ * o.y_ + z_ * o.z_; }
    float magnitude() const { return std::sqrt(dot(*this)); }

private:
    float x_, y_, z_;
};
}  // namespace tap::algorithms::transforms

#endif  // TAPROOT_VECTOR_HPP_
