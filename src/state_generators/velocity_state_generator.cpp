#include "state_generators/velocity_state_generator.hpp"

#include <algorithm>
#include <cmath>

#include "indexer.hpp"

namespace autonav::state_generators {

void VelocityStateGenerator::Generate(Path& path, const Indexer& indexer)
    const {
    using P = PositionStateGenerator;
    const size_t n = path.Size();
    const uint8_t ix = indexer.GetIndex(P::kId, P::kX);
    const uint8_t iy = indexer.GetIndex(P::kId, P::kY);
    const uint8_t itheta = indexer.GetIndex(P::kId, P::kTheta);
    const auto& s = path.ArcLength();
    auto xy = [&](size_t i) {
        return path.GetStateVector(i).segment<2>(std::min<size_t>(ix, iy));
    };
    static_assert(P::kY == P::kX + 1);

    // Curvature at each sample, from the circle through it and its neighbors.
    std::vector<double> speed(n, limits_.max_speed);
    for (size_t i = 1; i + 1 < n; ++i) {
        const Eigen::Vector2d a = xy(i - 1), b = xy(i), c = xy(i + 1);
        const double ab = (b - a).norm(), bc = (c - b).norm(),
                     ca = (a - c).norm();
        const double cross =
            std::abs((b - a).x() * (c - a).y() - (b - a).y() * (c - a).x());
        const double curvature =
            ab * bc * ca > 1e-12 ? 2.0 * cross / (ab * bc * ca) : 0.0;
        if (curvature > 1e-9) {
            speed[i] =
                std::min(speed[i], std::sqrt(limits_.max_lateral / curvature));
        }
    }
    speed.front() = 0.0;
    speed.back() = std::min(speed.back(), limits_.end_speed);

    // Forward pass (acceleration), then backward pass (braking).
    for (size_t i = 1; i < n; ++i) {
        const double d = s[i] - s[i - 1];
        speed[i] = std::min(
            speed[i],
            std::sqrt(
                speed[i - 1] * speed[i - 1] + 2.0 * limits_.max_acceleration * d
            )
        );
    }
    for (size_t i = n - 1; i-- > 0;) {
        const double d = s[i + 1] - s[i];
        speed[i] = std::min(
            speed[i],
            std::sqrt(
                speed[i + 1] * speed[i + 1] + 2.0 * limits_.max_acceleration * d
            )
        );
    }
    // The first sample starts from rest but must move off it.
    if (n > 1) {
        speed.front() = std::min(speed[1], 1e-3);
    }

    const uint8_t ivx = indexer.GetIndex(*this, kXDot);
    const uint8_t ivy = indexer.GetIndex(*this, kYDot);
    const uint8_t iw = indexer.GetIndex(*this, kThetaDot);
    for (size_t i = 0; i < n; ++i) {
        const size_t i0 = i == 0 ? 0 : i - 1, i1 = i + 1 == n ? i : i + 1;
        const Eigen::Vector2d tangent = (xy(i1) - xy(i0)).normalized();
        const double ds = s[i1] - s[i0];
        const double dtheta_ds = ds > 0.0 ? (path.GetStateVector(i1)[itheta] -
                                             path.GetStateVector(i0)[itheta]) /
                                                ds
                                          : 0.0;
        path.SetSpeed(i, speed[i]);
        path.SetState(i, ivx, speed[i] * tangent.x());
        path.SetState(i, ivy, speed[i] * tangent.y());
        path.SetState(i, iw, speed[i] * dtheta_ds);
    }
}

}  // namespace autonav::state_generators
