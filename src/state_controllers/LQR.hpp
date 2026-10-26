#ifndef LQR_PATHING_HPP_
#define LQR_PATHING_HPP_

#include <eigen3/Eigen/Dense>

#include "state_controllers/i_state_controller.hpp"

namespace autonav::state_controllers {

// Step 2 of the plan: infinite-horizon LQR for x_dot = A x + B u, minimizing
// the integral of e^T Q e + u^T R u. The gain K comes from the continuous
// algebraic Riccati equation, solved once at construction; the control is
// u = -K (x - x_ref).
class LQR : public IStateController {
  public:
    LQR(const Eigen::MatrixXd& a,
        const Eigen::MatrixXd& b,
        const Eigen::MatrixXd& q,
        const Eigen::MatrixXd& r);

    Eigen::VectorXd ComputeControl(const Eigen::VectorXd& error) const override;

    const Eigen::MatrixXd& Gain() const { return k_; }

    // The stabilizing solution P of A^T P + P A - P B R^-1 B^T P + Q = 0, from
    // the stable invariant subspace of the Hamiltonian matrix.
    static Eigen::MatrixXd SolveRiccati(
        const Eigen::MatrixXd& a,
        const Eigen::MatrixXd& b,
        const Eigen::MatrixXd& q,
        const Eigen::MatrixXd& r
    );

    // The plan's model: world-frame position (x, y, theta) and velocity, driven
    // by force (F_x, F_y) and torque on a body of mass `mass` and yaw inertia
    // `inertia`. In the world frame A and B are constant.
    static Eigen::MatrixXd ModelA();
    static Eigen::MatrixXd ModelB(double mass, double inertia);

  private:
    Eigen::MatrixXd a_;
    Eigen::MatrixXd b_;
    Eigen::MatrixXd k_;
};
}  // namespace autonav::state_controllers

#endif  // LQR_PATHING_HPP_
