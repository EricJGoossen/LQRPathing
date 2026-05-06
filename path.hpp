#ifndef PATH_HPP_
#define PATH_HPP_

#include <stdexcept>
#include <vector>

namespace auto_nav {
class Path {
  public:
    using StateType = double;

    Path(std::vector<std::vector<StateType>> waypoints)
      : waypoints_(std::move(waypoints)) {}

    const std::vector<StateType>& GetWaypoint(int waypoint_index) const {
        if (waypoint_index < 0 || waypoint_index >= waypoints_.size()) {
            throw std::out_of_range("Waypoint index out of range");
        }
        return waypoints_[waypoint_index];
    }

    void TimeParametrize();  // TODO: Implement this function to perform time
                             // parametrization on the waypoints

  private:
    std::vector<std::vector<StateType>> waypoints_;
};
}  // namespace auto_nav

#endif  // PATH_HPP_