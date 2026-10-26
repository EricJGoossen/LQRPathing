#ifndef POSITION_STATE_GENERATOR_HPP_
#define POSITION_STATE_GENERATOR_HPP_

#include <cstdint>
#include <vector>

#include "state_generators/abstract_state_generator.hpp"

namespace autonav::state_generators {

// Step 1 of the plan, positions: a cubic spline through the waypoints,
// reparametrized by arc length and sampled every `spacing` meters. Heading is
// interpolated between the waypoints' headings along the arc length (the
// chassis is holonomic, so it doesn't have to face along the path).
class PositionStateGenerator : public AbstractStateGenerator {
  public:
    enum State : std::uint8_t { kX = 0, kY = 1, kTheta = 2, kDimensions = 3 };

    explicit PositionStateGenerator(double spacing = 0.01)
      : spacing_(spacing) {}
    PositionStateGenerator(const PositionStateGenerator&) = delete;
    PositionStateGenerator& operator=(const PositionStateGenerator&) = delete;
    PositionStateGenerator(PositionStateGenerator&&) = default;
    PositionStateGenerator& operator=(PositionStateGenerator&&) = default;
    constexpr virtual ~PositionStateGenerator() override {}

    std::vector<int> Dependencies() const override { return dependencies_; }
    int Id() const override { return kId; }
    int Dimension() const override { return State::kDimensions; }

    void Generate(Path& current_path, const Indexer& indexer) const override;

    static constexpr int kId = 1;

  private:
    std::vector<int> dependencies_ = {};
    double spacing_;
};
}  // namespace autonav::state_generators

#endif  // POSITION_STATE_GENERATOR_HPP_
