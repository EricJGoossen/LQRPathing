#ifndef PATH_HPP_
#define PATH_HPP_

#include <vector>
#include <cstdint>
#include <eigen3/Eigen/Dense>

#include "state_type.hpp"

namespace autonav {
class Path {
  public:

    const Eigen::VectorXd& GetStateVector(uint8_t waypoint_index) const;

    void SetState(uint8_t waypoint_index, uint8_t state_index, StateType value);

    void TimeParametrize();  // TODO: Implement this function to perform time
                             // parametrization on the waypoints

  private:
    friend class PathGenerator;

    std::vector<Eigen::VectorXd> path_states_;
};
}  // namespace autonav

#endif  // PATH_HPP_