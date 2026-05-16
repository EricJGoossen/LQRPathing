#include "LQR.hpp"

namespace autonav::state_controllers {

LQR::LQR(const Eigen::MatrixXd& a,
         const Eigen::MatrixXd& b,
         const Eigen::MatrixXd& k)
    : a_(&a), b_(&b), k_(&k) {}

Eigen::VectorXd LQR::ComputeControl(const Eigen::VectorXd& x) const {
    Eigen::VectorXd u = -(*k_) * x;
    Eigen::VectorXd x_dot = (*a_) * x + (*b_) * u;

    return x_dot;
}

} // namespace autonav::state_controllers