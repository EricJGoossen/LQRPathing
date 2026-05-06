#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include <vector>

#include "state_generators/abstract_state_generator.hpp"
#include "topological_sort.hpp"

namespace auto_nav
{
template <typename... Generators>
class PathGenerator {
    using Generator = state_generators::AbstractStateGenerator;
public:
    PathGenerator() : generator_instances_(CreateGeneratorInstances()) {}
    PathGenerator(const PathGenerator&) = delete;
    PathGenerator& operator=(const PathGenerator&) = delete;
    PathGenerator(PathGenerator&&) = default;
    PathGenerator& operator=(PathGenerator&&) = default;
    ~PathGenerator() { 
        for (auto* g : generator_instances_) {
            delete g;
        }
    }

    const std::vector<Generator*>& GetGeneratorInstances() const {
        return generator_instances_;
    }

private:
    using Sorted = TopologicalSort::SortResult<Generators...>;

    static constexpr int NUM_GENERATORS = Sorted::Size();

    std::vector<Generator*> generator_instances_;

    static std::vector<Generator*> CreateGeneratorInstances() {
        return Sorted::template CreateInstancesFromTypeList<Generator>();
    }
};
}  // namespace auto_nav::state_generators

#endif  // PATH_GENERATOR_HPP_