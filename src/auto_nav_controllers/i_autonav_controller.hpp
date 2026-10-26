#ifndef I_AUTONAV_CONTROLLER_HPP_
#define I_AUTONAV_CONTROLLER_HPP_

#include <vector>

#include "state_type.hpp"

namespace autonav::autonav_controllers {
class IStateController {
  public:
    IStateController() = default;
    IStateController(const IStateController&) = default;
    IStateController& operator=(const IStateController&) = default;
    IStateController(IStateController&&) = default;
    IStateController& operator=(IStateController&&) = default;
    virtual ~IStateController() = default;

    virtual void Step(double dt) = 0;
    virtual void SetPathTarget(const std::vector<StateType>& path_target) = 0;
    virtual std::vector<StateType> GetControl() const = 0;
};
}  // namespace autonav::autonav_controllers

#endif  // I_AUTONAV_CONTROLLER_HPP_
