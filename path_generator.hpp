#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include <vector>
#include <memory>

#include "state_generators/abstract_state_generator.hpp"
#include "state_indexer.hpp"
#include "path.hpp"

namespace auto_nav
{
template <typename... Generators>
class PathGenerator {
    using StateType = Path::StateType;
    using Generator = state_generators::AbstractStateGenerator;

public:
    PathGenerator(Path initial_path)
        : generator_instances_(CreateGeneratorInstances()),
          state_indexer_(CreateStateIndexer()),
          path_(std::move(initial_path)) {}

    const StateIndexer& GetStateIndexer() const { return state_indexer_; }
    const Path& GetPath() const { return path_; }

    void GeneratePath(const std::vector<StateType>& current_state); // TODO: Implement this function to generate the path using the generator instances and state indexer

private:
    std::vector<std::shared_ptr<Generator>> generator_instances_;
    StateIndexer state_indexer_;
    Path path_;

    static std::vector<std::shared_ptr<Generator>> CreateGeneratorInstances();
    static StateIndexer CreateStateIndexer();
};
}  // namespace auto_nav

#endif  // PATH_GENERATOR_HPP_