#include "path.hpp"

#include <cstdint>

namespace autonav {

const Eigen::VectorXd& Path::GetStateVector(uint8_t waypoint_index) const {
    return path_states_[waypoint_index];
}

void Path::SetState(uint8_t waypoint_index, uint8_t state_index, StateType value) {
    path_states_[waypoint_index][state_index] = value;
}

} // namespace autonav