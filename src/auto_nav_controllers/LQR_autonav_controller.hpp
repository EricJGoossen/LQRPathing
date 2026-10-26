#ifndef LQR_AUTONAV_CONTROLLER_HPP_
#define LQR_AUTONAV_CONTROLLER_HPP_

#include <eigen3/Eigen/Dense>
#include <optional>
#include <vector>

#include "auto_nav_controllers/i_autonav_controller.hpp"
#include "indexer.hpp"
#include "path.hpp"
#include "state_controllers/LQR.hpp"
#include "state_type.hpp"

namespace autonav::autonav_controllers {

// Follows a generated path with an LQR. Each Step advances the reference along
// the path's time parametrization, computes the world-frame force and torque
// u = -K (x - x_ref), and rotates the force into the robot frame for the
// chassis.
class LQRAutonavController : public IStateController {
  public:
    LQRAutonavController(
        const Path& path,
        const Indexer& indexer,
        const state_controllers::LQR& lqr
    );

    // The robot's state in the world frame: x, y, theta, x_dot, y_dot,
    // theta_dot.
    void SetRobotState(const Eigen::VectorXd& state) { robot_state_ = state; }

    void Step(double dt) override;

    // Hold a fixed target state (x, y, theta, and optionally the velocities)
    // instead of following the path.
    void SetPathTarget(const std::vector<StateType>& path_target) override;

    // Robot-frame force (F_x, F_y) and torque.
    std::vector<StateType> GetControl() const override { return control_; }

    // The world-frame target the last Step tracked, in the controller's order.
    const Eigen::VectorXd& Reference() const { return reference_; }
    double Time() const { return time_; }
    bool Finished() const { return time_ >= path_->Duration(); }

  private:
    Eigen::VectorXd ReferenceAt(double t) const;

    const Path* path_;
    const Indexer* indexer_;
    const state_controllers::LQR* lqr_;
    double time_ = 0.0;
    Eigen::VectorXd robot_state_ = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd reference_ = Eigen::VectorXd::Zero(6);
    std::optional<Eigen::VectorXd> hold_;
    std::vector<StateType> control_ = {0.0, 0.0, 0.0};
};
}  // namespace autonav::autonav_controllers

#endif  // LQR_AUTONAV_CONTROLLER_HPP_
