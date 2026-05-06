#ifndef VELOCITY_STATE_GENERATOR_HPP_
#define VELOCITY_STATE_GENERATOR_HPP_

#include "../topological_sort.hpp"
#include "abstract_state_generator.hpp"
#include "position_state_generator.hpp"

namespace auto_nav::state_generators {
class VelocityStateGenerator : public AbstractStateGenerator {
  public:
    using Dependencies =
        TopologicalSort::DependenciesList<PositionStateGenerator>;

    enum State : std::uint8_t { kXDot, kYDot, kThetaDot, kDimensions };

    VelocityStateGenerator(const VelocityStateGenerator&) = delete;
    VelocityStateGenerator operator=(const VelocityStateGenerator&) = delete;
    VelocityStateGenerator(VelocityStateGenerator&&) = default;
    VelocityStateGenerator& operator=(VelocityStateGenerator&&) = default;
    constexpr virtual ~VelocityStateGenerator() override {}

    int Id() const override { return kId; }
    int Dimension() const override { return State::kDimensions; }

    void Generate(const StateIndexer& indexer, Path& current_path) const
        override;  // TODO: Implement this function to generate the velocity
                   // states based on the position states in the current path

    static constexpr int kId = 1;
};
}  // namespace auto_nav::state_generators

#endif  // VELOCITY_STATE_GENERATOR_HPP_