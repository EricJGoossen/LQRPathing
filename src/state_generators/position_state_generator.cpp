#include "state_generators/position_state_generator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "indexer.hpp"

namespace autonav::state_generators {
namespace {

// A natural cubic spline through (u_i, y_i): y(u) on each interval is a cubic
// with matching first and second derivatives at the knots, and zero second
// derivative at the ends.
class CubicSpline {
  public:
    CubicSpline(const std::vector<double>& u, const std::vector<double>& y) : u_(u), y_(y) {
        const size_t n = u.size();
        m_.assign(n, 0.0);  // second derivatives at the knots
        if (n < 3) {
            return;
        }
        // Tridiagonal system for the interior second derivatives (Thomas algorithm).
        std::vector<double> a(n, 0.0), b(n, 1.0), c(n, 0.0), d(n, 0.0);
        for (size_t i = 1; i + 1 < n; ++i) {
            const double h0 = u[i] - u[i - 1];
            const double h1 = u[i + 1] - u[i];
            a[i] = h0;
            b[i] = 2.0 * (h0 + h1);
            c[i] = h1;
            d[i] = 6.0 * ((y[i + 1] - y[i]) / h1 - (y[i] - y[i - 1]) / h0);
        }
        for (size_t i = 1; i < n; ++i) {
            const double w = a[i] / b[i - 1];
            b[i] -= w * c[i - 1];
            d[i] -= w * d[i - 1];
        }
        m_[n - 1] = d[n - 1] / b[n - 1];
        for (size_t i = n - 1; i-- > 0;) {
            m_[i] = (d[i] - c[i] * m_[i + 1]) / b[i];
        }
    }

    double operator()(double u) const {
        size_t i = Interval(u);
        const double h = u_[i + 1] - u_[i];
        const double a = (u_[i + 1] - u) / h;
        const double b = (u - u_[i]) / h;
        return a * y_[i] + b * y_[i + 1] +
               ((a * a * a - a) * m_[i] + (b * b * b - b) * m_[i + 1]) * h * h / 6.0;
    }

  private:
    size_t Interval(double u) const {
        size_t i = 0;
        while (i + 2 < u_.size() && u > u_[i + 1]) {
            ++i;
        }
        return i;
    }

    std::vector<double> u_, y_, m_;
};

double Unwrap(double angle, double reference) {
    return reference + std::remainder(angle - reference, 2.0 * M_PI);
}
}  // namespace

void PositionStateGenerator::Generate(Path& current_path, const Indexer& indexer) const {
    const auto& waypoints = current_path.Waypoints();
    if (waypoints.size() < 2) {
        throw std::invalid_argument("A path needs at least two waypoints");
    }

    // Spline parameter: chord length between waypoints.
    std::vector<double> u{0.0}, xs{waypoints[0].x()}, ys{waypoints[0].y()};
    for (size_t i = 1; i < waypoints.size(); ++i) {
        u.push_back(u.back() + (waypoints[i].head<2>() - waypoints[i - 1].head<2>()).norm());
        xs.push_back(waypoints[i].x());
        ys.push_back(waypoints[i].y());
    }
    const CubicSpline spline_x(u, xs), spline_y(u, ys);

    // Arc length as a function of the spline parameter, from a dense sweep; then
    // invert it to place samples every `spacing_` meters along the curve.
    const size_t dense = 200 * (waypoints.size() - 1);
    std::vector<double> dense_u(dense + 1), dense_s(dense + 1, 0.0);
    double px = spline_x(0.0), py = spline_y(0.0);
    for (size_t k = 0; k <= dense; ++k) {
        dense_u[k] = u.back() * static_cast<double>(k) / static_cast<double>(dense);
        const double qx = spline_x(dense_u[k]), qy = spline_y(dense_u[k]);
        if (k > 0) {
            dense_s[k] = dense_s[k - 1] + std::hypot(qx - px, qy - py);
        }
        px = qx;
        py = qy;
    }
    const double length = dense_s.back();
    const size_t samples = static_cast<size_t>(std::ceil(length / spacing_)) + 1;
    current_path.Resize(samples, indexer.Dimension());

    // Heading: unwrapped waypoint headings, placed at each waypoint's arc length.
    std::vector<double> waypoint_s(waypoints.size()), waypoint_theta(waypoints.size());
    for (size_t i = 0, k = 0; i < waypoints.size(); ++i) {
        while (k < dense && dense_u[k] < u[i]) {
            ++k;
        }
        waypoint_s[i] = dense_s[k];
        waypoint_theta[i] = i == 0 ? waypoints[0].z() : Unwrap(waypoints[i].z(), waypoint_theta[i - 1]);
    }

    const uint8_t ix = indexer.GetIndex(*this, kX);
    const uint8_t iy = indexer.GetIndex(*this, kY);
    const uint8_t itheta = indexer.GetIndex(*this, kTheta);
    for (size_t j = 0, k = 0, w = 0; j < samples; ++j) {
        const double s = std::min(length, spacing_ * static_cast<double>(j));
        while (k + 1 < dense && dense_s[k + 1] < s) {
            ++k;
        }
        const double span = dense_s[k + 1] - dense_s[k];
        const double a = span > 0.0 ? (s - dense_s[k]) / span : 0.0;
        const double us = dense_u[k] + a * (dense_u[k + 1] - dense_u[k]);
        while (w + 2 < waypoints.size() && s > waypoint_s[w + 1]) {
            ++w;
        }
        const double ws = waypoint_s[w + 1] - waypoint_s[w];
        const double b = ws > 0.0 ? std::clamp((s - waypoint_s[w]) / ws, 0.0, 1.0) : 1.0;

        current_path.SetArcLength(j, s);
        current_path.SetState(j, ix, spline_x(us));
        current_path.SetState(j, iy, spline_y(us));
        current_path.SetState(j, itheta, waypoint_theta[w] + b * (waypoint_theta[w + 1] - waypoint_theta[w]));
    }
}

}  // namespace autonav::state_generators
