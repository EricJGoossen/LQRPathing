#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include <vector>

#include "path.hpp"
#include "state_generators/abstract_state_generator.hpp"
#include "indexer.hpp"

namespace autonav {

class PathGenerator {
    using Generator = state_generators::AbstractStateGenerator;

  public:
    PathGenerator(std::vector<const Generator*>& generators);

    void GeneratePath(Path& waypoint_path);

  private:
    void InitializeGenerators();

    void TopologicalSort();

    std::vector<const Generator*>* generators_;
    Indexer indexer_;
};
}  // namespace autonav

#endif  // PATH_GENERATOR_HPP_