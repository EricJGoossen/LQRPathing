#ifndef VELOCITY_STATE_GENERATOR_HPP_
#define VELOCITY_STATE_GENERATOR_HPP_

#include "state_generators/abstract_state_generator.hpp"
#include "state_generators/position_state_generator.hpp"

namespace autonav::state_generators {

// Step 1 of the plan, velocities: the fastest speed profile the robot can
// actually follow. At each sample the speed is capped by the top speed and by
// the curvature, v <= sqrt(a_lateral / k); a forward pass then limits how fast
// it can speed up, v_{n+1} <= sqrt(v_n^2 + 2 a d), and a backward pass how fast
// it can slow down, so it brakes before a tight corner and stops at the end.
// The states are the world-frame velocity along the path and the heading rate.
struct VelocityLimits {
    double max_speed = 2.0;         // m/s
    double max_acceleration = 3.0;  // m/s^2, along the path
    double max_lateral = 3.0;       // m/s^2, across it (sets the corner speed)
    double end_speed = 0.0;         // m/s at the last waypoint
};

class VelocityStateGenerator : public AbstractStateGenerator {
  public:
    enum State : std::uint8_t {
        kXDot = 0,
        kYDot = 1,
        kThetaDot = 2,
        kDimensions = 3
    };

    using Limits = VelocityLimits;

    explicit VelocityStateGenerator(Limits limits = Limits())
      : limits_(limits) {}
    VelocityStateGenerator(const VelocityStateGenerator&) = delete;
    VelocityStateGenerator& operator=(const VelocityStateGenerator&) = delete;
    VelocityStateGenerator(VelocityStateGenerator&&) = default;
    VelocityStateGenerator& operator=(VelocityStateGenerator&&) = default;
    constexpr virtual ~VelocityStateGenerator() override {}

    std::vector<int> Dependencies() const override { return dependencies_; }
    int Id() const override { return kId; }
    int Dimension() const override { return State::kDimensions; }

    void Generate(Path& current_path, const Indexer& indexer) const override;

    static constexpr int kId = 2;

  private:
    std::vector<int> dependencies_ = {PositionStateGenerator::kId};
    Limits limits_;
};
}  // namespace autonav::state_generators

#endif  // VELOCITY_STATE_GENERATOR_HPP_
