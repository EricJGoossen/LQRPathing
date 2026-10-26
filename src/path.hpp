#ifndef PATH_HPP_
#define PATH_HPP_

#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "state_type.hpp"

namespace autonav {

// A path: the waypoints it was asked to pass through, and, once generated, a
// dense sequence of target states sampled along it (one state vector per
// sample, laid out by the PathGenerator's Indexer), each with its arc length
// and, after TimeParametrize(), the time the robot should reach it.
class Path {
  public:
    // Waypoints are (x, y, theta) in the world frame.
    explicit Path(std::vector<Eigen::Vector3d> waypoints = {});

    const std::vector<Eigen::Vector3d>& Waypoints() const { return waypoints_; }
    void SetWaypoints(std::vector<Eigen::Vector3d> waypoints);

    size_t Size() const { return path_states_.size(); }

    const Eigen::VectorXd& GetStateVector(size_t sample_index) const;

    void SetState(size_t sample_index, uint8_t state_index, StateType value);

    // Arc length along the path at each sample (written by the position
    // generator).
    const std::vector<double>& ArcLength() const { return arc_length_; }
    // Target speed along the path at each sample (written by the velocity
    // generator).
    const std::vector<double>& Speed() const { return speed_; }
    void SetSpeed(size_t sample_index, double speed) {
        speed_[sample_index] = speed;
    }

    // Time at each sample, from the arc length and the speed profile: between
    // two samples the speed is taken to change uniformly, so dt = 2 ds / (v0 +
    // v1).
    void TimeParametrize();
    const std::vector<double>& Time() const { return time_; }
    double Duration() const { return time_.empty() ? 0.0 : time_.back(); }

    // The target state at time t (linear between samples, held after the end).
    Eigen::VectorXd Sample(double t) const;

    // Used by the state generators to lay out and fill the samples.
    void Resize(size_t samples, int dimension);
    void SetArcLength(size_t sample_index, double s) {
        arc_length_[sample_index] = s;
    }

  private:
    std::vector<Eigen::Vector3d> waypoints_;
    std::vector<Eigen::VectorXd> path_states_;
    std::vector<double> arc_length_;
    std::vector<double> speed_;
    std::vector<double> time_;
    int dimension_ = 0;
};
}  // namespace autonav

#endif  // PATH_HPP_
