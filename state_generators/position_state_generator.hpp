#ifndef POSITION_STATE_GENERATOR_HPP_
#define POSITION_STATE_GENERATOR_HPP_

#include <cstdint>

#include "../topological_sort.hpp"
#include "abstract_state_generator.hpp"

namespace auto_nav::state_generators {

class PositionStateGenerator : public AbstractStateGenerator {
  public:
    using Dependencies = TopologicalSort::DependenciesList<>;

    enum State : std::uint8_t { kX = 0, kY = 1, kTheta = 2, kDimensions = 3 };

    PositionStateGenerator(const PositionStateGenerator&) = delete;
    PositionStateGenerator operator=(const PositionStateGenerator&) = delete;
    PositionStateGenerator(PositionStateGenerator&&) = default;
    PositionStateGenerator& operator=(PositionStateGenerator&&) = default;

    constexpr virtual ~PositionStateGenerator() override {}
    int Id() const override { return kId; }
    int Dimension() const override { return State::kDimensions; }

    void Generate(const StateIndexer& indexer, Path& current_path)
        const override;  // TODO: Implement this function to generate the
                         // position states in the current path

    static constexpr int kId = 1;
};
}  // namespace auto_nav::state_generators

#endif  // POSITION_STATE_GENERATOR_HPP_