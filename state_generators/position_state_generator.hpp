#ifndef POSITION_STATE_GENERATOR_HPP_
#define POSITION_STATE_GENERATOR_HPP_

#include "abstract_state_generator.hpp"
#include "../topological_sort.hpp"

namespace auto_nav::state_generators
{
class PositionStateGenerator : public AbstractStateGenerator
{
public:
    constexpr virtual ~PositionStateGenerator() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<>;
    static constexpr int ID = 1;
};
}  // namespace auto_nav::state_generators

#endif  // POSITION_STATE_GENERATOR_HPP_