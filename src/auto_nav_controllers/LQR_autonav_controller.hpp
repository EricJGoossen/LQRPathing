#ifndef LQR_AUTONAV_CONTROLLER_HPP_
#define LQR_AUTONAV_CONTROLLER_HPP_

#include <vector>

#include "auto_nav_controllers/i_autonav_controller.hpp"
#include "state_type.hpp"

namespace autonav::autonav_controllers {
class LQRAutonavController : public IStateController {
  public:
    LQRAutonavController(const LQRAutonavController&) = default;
    LQRAutonavController& operator=(const LQRAutonavController&) = default;
    LQRAutonavController(LQRAutonavController&&) = default;
    LQRAutonavController& operator=(LQRAutonavController&&) = default;
    ~LQRAutonavController() = default;

    void Step(double dt) override;
    void SetPathTarget(const std::vector<StateType>& path_target) override;
    std::vector<StateType> GetControl() const override;
};
}  // namespace autonav::autonav_controllers

#endif  // LQR_AUTONAV_CONTROLLER_HPP_