// LQR path tracking against pure pursuit, on the same simulated chassis.
//
// The chassis: a holonomic body in the plane (x, y, heading) with mass M and
// yaw inertia I, driven by a world-frame force and a torque, the force limited
// to M * kTraction (what the wheels can push before slipping) and the torque to
// I * kMaxAngularAccel. Mass, inertia and the limits are picked, not measured.
//
// Pure pursuit is ARUW's ChassisAutoNavController, vendored unmodified from
// aruw-mcb in third_party/aruw-mcb and run against host stand-ins for the
// chassis, odometry transform and clock: it finds the robot's progress along a
// polyline path, aims at the point LOOKAHEAD_DISTANCE further along, and
// commands a chassis-frame wheel speed toward it; the chassis turns that into
// force through its wheel-speed loop (here a proportional loop with gain
// kWheelLoop). Beyblade and the cap bank are off. LQR follows the same path's
// time-parametrized states with this library's LQRAutonavController. Both get
// the same path: the generated spline, which pure pursuit receives as a
// polyline of its samples.
//
//   compare_pure_pursuit <x0 y0 x1 y1 ...>    (waypoints in meters)
// prints CSV: t, then x, y for pure pursuit, LQR and LQR's reference; then a
// summary line per controller: time to finish, RMS and max distance from the
// path.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "aruwsrc/algorithms/auto_nav_path.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include "auto_nav_controllers/LQR_autonav_controller.hpp"
#include "path.hpp"
#include "path_generator.hpp"
#include "state_controllers/LQR.hpp"
#include "state_generators/position_state_generator.hpp"
#include "state_generators/velocity_state_generator.hpp"

namespace {
constexpr double kMass = 20.0;             // kg
constexpr double kInertia = 0.6;           // kg m^2
constexpr double kTraction = 4.0;          // m/s^2 the wheels can deliver
constexpr double kMaxAngularAccel = 20.0;  // rad/s^2
constexpr double kDt = 0.002;              // 500 Hz, the MCB's control loop
constexpr double kTopSpeed = 2.0;          // m/s, both controllers

constexpr double kWheelLoop = 8.0;  // 1/s, pure pursuit's chassis velocity loop

struct Body {
    double x = 0, y = 0, th = 0, vx = 0, vy = 0, w = 0;
    void Apply(double fx, double fy, double tau) {
        const double f = std::hypot(fx, fy), fmax = kMass * kTraction;
        if (f > fmax) {
            fx *= fmax / f;
            fy *= fmax / f;
        }
        tau = std::clamp(
            tau, -kInertia * kMaxAngularAccel, kInertia * kMaxAngularAccel
        );
        vx += fx / kMass * kDt;
        vy += fy / kMass * kDt;
        w += tau / kInertia * kDt;
        x += vx * kDt;
        y += vy * kDt;
        th += w * kDt;
    }
};

struct Polyline {
    std::vector<double> x, y, s;
    double Length() const { return s.back(); }
    double Distance(double px, double py) const {
        double best = 1e9;
        for (size_t i = 1; i < s.size(); ++i) {
            const double dx = x[i] - x[i - 1], dy = y[i] - y[i - 1],
                         l2 = dx * dx + dy * dy;
            const double a =
                l2 > 0 ? std::clamp(
                             ((px - x[i - 1]) * dx + (py - y[i - 1]) * dy) / l2,
                             0.0,
                             1.0
                         )
                       : 0.0;
            best = std::min(
                best, std::hypot(px - x[i - 1] - a * dx, py - y[i - 1] - a * dy)
            );
        }
        return best;
    }
};
}  // namespace

int main(int argc, char** argv) {
    using namespace autonav;
    if (argc < 5 || (argc - 1) % 2) {
        std::fprintf(stderr, "usage: %s x0 y0 x1 y1 ...\n", argv[0]);
        return 1;
    }
    std::vector<Eigen::Vector3d> waypoints;
    for (int i = 1; i + 1 < argc; i += 2)
        waypoints.emplace_back(std::atof(argv[i]), std::atof(argv[i + 1]), 0.0);

    state_generators::PositionStateGenerator position(0.01);
    state_generators::VelocityStateGenerator::Limits limits;
    limits.max_speed = kTopSpeed;
    limits.max_acceleration = 2.5;  // a margin under the traction limit
    limits.max_lateral = 2.5;
    state_generators::VelocityStateGenerator velocity(limits);
    std::vector<const state_generators::AbstractStateGenerator*> generators = {
        &position, &velocity};
    PathGenerator generator(generators);
    Path path(waypoints);
    generator.GeneratePath(path);

    // Bryson's rule: 2 cm, 0.05 rad, 0.1 m/s and 0.3 rad/s of error are as
    // costly as 40 N of force or 5 N m of torque.
    const Eigen::MatrixXd q =
        Eigen::Vector<double, 6>(
            1 / 4e-4, 1 / 4e-4, 1 / 2.5e-3, 1 / 1e-2, 1 / 1e-2, 1 / 9e-2
        )
            .asDiagonal();
    const Eigen::MatrixXd r =
        Eigen::Vector3d(1.0 / 1600, 1.0 / 1600, 1.0 / 25).asDiagonal();
    const state_controllers::LQR lqr(
        state_controllers::LQR::ModelA(),
        state_controllers::LQR::ModelB(kMass, kInertia),
        q,
        r
    );
    autonav_controllers::LQRAutonavController follower(
        path, generator.GetIndexer(), lqr
    );

    Polyline line;
    for (size_t i = 0; i < path.Size(); i += 5) {
        line.x.push_back(path.GetStateVector(i)[0]);
        line.y.push_back(path.GetStateVector(i)[1]);
    }
    line.x.push_back(path.GetStateVector(path.Size() - 1)[0]);
    line.y.push_back(path.GetStateVector(path.Size() - 1)[1]);
    line.s.assign(line.x.size(), 0.0);
    for (size_t i = 1; i < line.x.size(); ++i)
        line.s[i] =
            line.s[i - 1] +
            std::hypot(line.x[i] - line.x[i - 1], line.y[i] - line.y[i - 1]);

    Body pp, lq;
    pp.x = lq.x = waypoints[0].x();
    pp.y = lq.y = waypoints[0].y();

    // Built the way the robot's commands build it: reset, then push points.
    // resetPath() marks the path changed, which the controller relies on to
    // start its transition timeout.
    aruwsrc::algorithms::AutoNavPath pp_path;
    pp_path.resetPath();
    for (size_t i = 0; i < line.x.size(); ++i)
        pp_path.pushPoint(
            tap::algorithms::transforms::Position(line.x[i], line.y[i], 0)
        );
    tap::Drivers drivers;
    aruwsrc::control::chassis::HolonomicChassisSubsystem chassis;
    tap::algorithms::transforms::Transform world_to_chassis(pp.x, pp.y, pp.th);
    const aruwsrc::control::chassis::BeybladeConfig beyblade{0, 1, 0, 1, 0};
    aruwsrc::control::chassis::ChassisAutoNavController pursuit(
        drivers, chassis, world_to_chassis, beyblade, nullptr, 0, 0
    );
    pursuit.attachPath(&pp_path);
    pursuit.setDesiredSpeed(kTopSpeed);
    pursuit.initialize();
    // Wheel RPM back to m/s, as the controller converts.
    const double rpm_to_speed =
        aruwsrc::control::chassis::WHEEL_RADIUS * 2 * M_PI / 60;

    double pp_done = -1, lq_done = -1;
    double pp_sq = 0, lq_sq = 0, pp_max = 0, lq_max = 0;
    int samples = 0;
    const double end_x = line.x.back(), end_y = line.y.back();

    std::printf("t,pp_x,pp_y,lqr_x,lqr_y,ref_x,ref_y\n");
    const double horizon = path.Duration() * 2.0 + 3.0;
    for (int k = 0; k * kDt < horizon; ++k) {
        const double t = k * kDt;
        // Pure pursuit: the commanded chassis-frame velocity, back to the
        // world frame. With beyblade off it commands no rotation.
        tap::arch::clock::setTimeMicroseconds(
            static_cast<uint32_t>(k * kDt * 1e6)
        );
        world_to_chassis.update(pp.x, pp.y, pp.th);
        pursuit.runController(0, true, false);
        const double bx = chassis.desiredX * rpm_to_speed,
                     by = chassis.desiredY * rpm_to_speed;
        const double cvx = std::cos(pp.th) * bx - std::sin(pp.th) * by,
                     cvy = std::sin(pp.th) * bx + std::cos(pp.th) * by;
        pp.Apply(
            kMass * kWheelLoop * (cvx - pp.vx),
            kMass * kWheelLoop * (cvy - pp.vy),
            kInertia * kWheelLoop * (0.0 - pp.w)
        );
        // LQR.
        Eigen::VectorXd state(6);
        state << lq.x, lq.y, lq.th, lq.vx, lq.vy, lq.w;
        follower.SetRobotState(state);
        follower.Step(kDt);
        const auto u =
            follower.GetControl();  // robot frame: back to the world frame
        const double c = std::cos(lq.th), s = std::sin(lq.th);
        lq.Apply(c * u[0] - s * u[1], s * u[0] + c * u[1], u[2]);

        const double dpp = line.Distance(pp.x, pp.y),
                     dlq = line.Distance(lq.x, lq.y);
        if (pp_done < 0) {
            pp_sq += dpp * dpp;
            pp_max = std::max(pp_max, dpp);
        }
        if (lq_done < 0) {
            lq_sq += dlq * dlq;
            lq_max = std::max(lq_max, dlq);
        }
        ++samples;
        auto finished = [&](const Body& b) {
            return std::hypot(b.x - end_x, b.y - end_y) < 0.02 &&
                   std::hypot(b.vx, b.vy) < 0.05;
        };
        if (pp_done < 0 && finished(pp)) pp_done = t;
        if (lq_done < 0 && finished(lq)) lq_done = t;
        if (k % 10 == 0) {
            const auto& ref = follower.Reference();
            std::printf(
                "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                t,
                pp.x,
                pp.y,
                lq.x,
                lq.y,
                ref[0],
                ref[1]
            );
        }
        if (pp_done >= 0 && lq_done >= 0 &&
            t > std::max(pp_done, lq_done) + 0.5)
            break;
    }
    (void)samples;
    const int n_pp = static_cast<int>((pp_done > 0 ? pp_done : horizon) / kDt);
    const int n_lq = static_cast<int>((lq_done > 0 ? lq_done : horizon) / kDt);
    std::printf(
        "# pure_pursuit finish %.2f s, rms %.1f cm, max %.1f cm\n",
        pp_done,
        100 * std::sqrt(pp_sq / std::max(1, n_pp)),
        100 * pp_max
    );
    std::printf(
        "# lqr finish %.2f s, rms %.1f cm, max %.1f cm, plan %.2f s, path %.2f "
        "m\n",
        lq_done,
        100 * std::sqrt(lq_sq / std::max(1, n_lq)),
        100 * lq_max,
        path.Duration(),
        line.Length()
    );
    return 0;
}
