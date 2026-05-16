#ifndef LQR_PATHING_HPP_
#define LQR_PATHING_HPP_

#include <eigen3/Eigen/Dense>

#include "state_controllers/i_state_controller.hpp"

namespace autonav::state_controllers {
class LQR : public IStateController {
  public:
    LQR(const Eigen::MatrixXd& a,
        const Eigen::MatrixXd& b,
        const Eigen::MatrixXd& k);

    Eigen::VectorXd ComputeControl(
        const Eigen::VectorXd& x
    ) const override;

  private:
    const Eigen::MatrixXd* a_;
    const Eigen::MatrixXd* b_;
    const Eigen::MatrixXd* k_;
};
}  // namespace autonav::state_controllers

#endif  // LQR_PATHING_HPP_