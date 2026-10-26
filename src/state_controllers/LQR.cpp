#include "LQR.hpp"

#include <algorithm>
#include <eigen3/Eigen/Eigenvalues>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace autonav::state_controllers {

LQR::LQR(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b, const Eigen::MatrixXd& q,
         const Eigen::MatrixXd& r)
    : a_(a), b_(b) {
    const Eigen::MatrixXd p = SolveRiccati(a, b, q, r);
    k_ = r.ldlt().solve(b.transpose() * p);
}

Eigen::VectorXd LQR::ComputeControl(const Eigen::VectorXd& error) const { return -k_ * error; }

Eigen::MatrixXd LQR::SolveRiccati(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                                  const Eigen::MatrixXd& q, const Eigen::MatrixXd& r) {
    const Eigen::Index n = a.rows();
    Eigen::MatrixXd h(2 * n, 2 * n);
    h << a, -b * r.ldlt().solve(b.transpose()), -q, -a.transpose();

    Eigen::ComplexEigenSolver<Eigen::MatrixXd> solver(h);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Riccati: eigendecomposition failed");
    }
    std::vector<Eigen::Index> order(2 * n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](Eigen::Index i, Eigen::Index j) {
        return solver.eigenvalues()[i].real() < solver.eigenvalues()[j].real();
    });
    Eigen::MatrixXcd stable(2 * n, n);
    for (Eigen::Index k = 0; k < n; ++k) {
        if (solver.eigenvalues()[order[k]].real() >= 0.0) {
            throw std::runtime_error("Riccati: system is not stabilizable");
        }
        stable.col(k) = solver.eigenvectors().col(order[k]);
    }
    const Eigen::MatrixXcd u1 = stable.topRows(n), u2 = stable.bottomRows(n);
    Eigen::MatrixXd p = (u2 * u1.inverse()).real();
    return 0.5 * (p + p.transpose());
}

Eigen::MatrixXd LQR::ModelA() {
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6, 6);
    a.topRightCorner(3, 3).setIdentity();
    return a;
}

Eigen::MatrixXd LQR::ModelB(double mass, double inertia) {
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6, 3);
    b(3, 0) = 1.0 / mass;
    b(4, 1) = 1.0 / mass;
    b(5, 2) = 1.0 / inertia;
    return b;
}

}  // namespace autonav::state_controllers
