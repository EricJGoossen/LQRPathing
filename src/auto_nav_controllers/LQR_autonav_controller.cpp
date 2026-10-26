#include "auto_nav_controllers/LQR_autonav_controller.hpp"

#include <cmath>

#include "state_generators/position_state_generator.hpp"
#include "state_generators/velocity_state_generator.hpp"

namespace autonav::autonav_controllers {

using state_generators::PositionStateGenerator;
using state_generators::VelocityStateGenerator;

LQRAutonavController::LQRAutonavController(
    const Path& path, const Indexer& indexer, const state_controllers::LQR& lqr
)
  : path_(&path), indexer_(&indexer), lqr_(&lqr) {}

Eigen::VectorXd LQRAutonavController::ReferenceAt(double t) const {
    if (hold_) {
        return *hold_;
    }
    const Eigen::VectorXd sample = path_->Sample(t);
    Eigen::VectorXd ref(6);
    ref << sample[indexer_->GetIndex(
        PositionStateGenerator::kId, PositionStateGenerator::kX
    )],
        sample[indexer_->GetIndex(
            PositionStateGenerator::kId, PositionStateGenerator::kY
        )],
        sample[indexer_->GetIndex(
            PositionStateGenerator::kId, PositionStateGenerator::kTheta
        )],
        sample[indexer_->GetIndex(
            VelocityStateGenerator::kId, VelocityStateGenerator::kXDot
        )],
        sample[indexer_->GetIndex(
            VelocityStateGenerator::kId, VelocityStateGenerator::kYDot
        )],
        sample[indexer_->GetIndex(
            VelocityStateGenerator::kId, VelocityStateGenerator::kThetaDot
        )];
    return ref;
}

void LQRAutonavController::Step(double dt) {
    time_ += dt;
    reference_ = ReferenceAt(time_);
    Eigen::VectorXd error = robot_state_ - reference_;
    error[2] = std::remainder(
        error[2], 2.0 * M_PI
    );  // heading error the short way round

    const Eigen::VectorXd u = lqr_->ComputeControl(error);  // world frame
    const double c = std::cos(robot_state_[2]), s = std::sin(robot_state_[2]);
    control_ = {c * u[0] + s * u[1], -s * u[0] + c * u[1], u[2]};
}

void LQRAutonavController::SetPathTarget(
    const std::vector<StateType>& path_target
) {
    Eigen::VectorXd target = Eigen::VectorXd::Zero(6);
    for (size_t i = 0; i < path_target.size() && i < 6; ++i) {
        target[static_cast<Eigen::Index>(i)] = path_target[i];
    }
    hold_ = target;
}

}  // namespace autonav::autonav_controllers
