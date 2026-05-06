#ifndef VELOCITY_STATE_GENERATOR_HPP_
#define VELOCITY_STATE_GENERATOR_HPP_

#include "abstract_state_generator.hpp"
#include "position_state_generator.hpp"
#include "../topological_sort.hpp"

namespace auto_nav::state_generators
{
class VelocityStateGenerator : public AbstractStateGenerator
{
public:
    constexpr virtual ~VelocityStateGenerator() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<PositionStateGenerator>;
    static constexpr int ID = 2;
};
}  // namespace auto_nav::state_generators

#endif  // VELOCITY_STATE_GENERATOR_HPP_