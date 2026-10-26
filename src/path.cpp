#include "path.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace autonav {

Path::Path(std::vector<Eigen::Vector3d> waypoints) : waypoints_(std::move(waypoints)) {}

void Path::SetWaypoints(std::vector<Eigen::Vector3d> waypoints) {
    waypoints_ = std::move(waypoints);
    path_states_.clear();
    arc_length_.clear();
    speed_.clear();
    time_.clear();
}

const Eigen::VectorXd& Path::GetStateVector(size_t sample_index) const {
    return path_states_[sample_index];
}

void Path::SetState(size_t sample_index, uint8_t state_index, StateType value) {
    path_states_[sample_index][state_index] = value;
}

void Path::Resize(size_t samples, int dimension) {
    if (path_states_.size() == samples && dimension_ == dimension) {
        return;
    }
    dimension_ = dimension;
    path_states_.assign(samples, Eigen::VectorXd::Zero(dimension));
    arc_length_.assign(samples, 0.0);
    speed_.assign(samples, 0.0);
    time_.assign(samples, 0.0);
}

void Path::TimeParametrize() {
    if (path_states_.empty()) {
        return;
    }
    time_[0] = 0.0;
    for (size_t i = 1; i < path_states_.size(); ++i) {
        const double ds = arc_length_[i] - arc_length_[i - 1];
        const double v = speed_[i - 1] + speed_[i];
        if (v <= 1e-9) {
            throw std::runtime_error("TimeParametrize: zero speed inside the path");
        }
        time_[i] = time_[i - 1] + 2.0 * ds / v;
    }
}

Eigen::VectorXd Path::Sample(double t) const {
    if (t <= time_.front()) {
        return path_states_.front();
    }
    if (t >= time_.back()) {
        return path_states_.back();
    }
    const auto it = std::upper_bound(time_.begin(), time_.end(), t);
    const size_t i = static_cast<size_t>(it - time_.begin());
    const double a = (t - time_[i - 1]) / (time_[i] - time_[i - 1]);
    return (1.0 - a) * path_states_[i - 1] + a * path_states_[i];
}

}  // namespace autonav
