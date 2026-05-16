#ifndef I_STATE_CONTROLLER_HPP_
#define I_STATE_CONTROLLER_HPP_

#include <eigen3/Eigen/Dense>

namespace autonav::state_controllers {
class IStateController {
  public:
    IStateController();
    IStateController(const IStateController&) = default;
    IStateController& operator=(const IStateController&) = default;
    IStateController(IStateController&&) = default;
    IStateController& operator=(IStateController&&) = default;
    virtual ~IStateController() = default;

    virtual Eigen::VectorXd ComputeControl(
        const Eigen::VectorXd& x
    ) const = 0;
};
}  // namespace autonav::state_controllers

#endif  // I_STATE_CONTROLLER_HPP_