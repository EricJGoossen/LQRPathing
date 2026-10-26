// A small demo: generate a path through a few waypoints, print its speed
// profile, and the LQR gain for the plan's model.
#include <cstdio>
#include <vector>

#include "path.hpp"
#include "path_generator.hpp"
#include "state_controllers/LQR.hpp"
#include "state_generators/abstract_state_generator.hpp"
#include "state_generators/position_state_generator.hpp"
#include "state_generators/velocity_state_generator.hpp"

int main() {
    using namespace autonav;
    state_generators::PositionStateGenerator position(0.01);
    state_generators::VelocityStateGenerator velocity;
    std::vector<const state_generators::AbstractStateGenerator*> generators = {
        &velocity, &position};
    PathGenerator generator(generators);

    Path path(
        {{0.0, 0.0, 0.0}, {1.5, 0.3, 0.0}, {2.0, 1.5, 0.0}, {0.5, 2.0, 0.0}}
    );
    generator.GeneratePath(path);
    std::printf(
        "%zu samples, %.2f m, %.2f s\n",
        path.Size(),
        path.ArcLength().back(),
        path.Duration()
    );
    for (size_t i = 0; i < path.Size(); i += path.Size() / 10) {
        const auto& x = path.GetStateVector(i);
        std::printf(
            "  t %5.2f  s %5.2f  v %4.2f  (%5.2f, %5.2f)\n",
            path.Time()[i],
            path.ArcLength()[i],
            path.Speed()[i],
            x[0],
            x[1]
        );
    }

    const Eigen::MatrixXd q =
        Eigen::Vector<double, 6>(2500, 2500, 400, 100, 100, 11).asDiagonal();
    const Eigen::MatrixXd r =
        Eigen::Vector3d(1.0 / 1600, 1.0 / 1600, 1.0 / 25).asDiagonal();
    const state_controllers::LQR lqr(
        state_controllers::LQR::ModelA(),
        state_controllers::LQR::ModelB(20.0, 0.6),
        q,
        r
    );
    std::printf("K =\n");
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) std::printf(" %8.2f", lqr.Gain()(i, j));
        std::printf("\n");
    }
    return 0;
}
