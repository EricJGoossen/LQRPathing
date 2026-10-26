#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include <vector>

#include "indexer.hpp"
#include "path.hpp"
#include "state_generators/abstract_state_generator.hpp"

namespace autonav {

// Fills a Path's target states by running its state generators in dependency
// order (position before velocity, and so on), then times it.
class PathGenerator {
    using Generator = state_generators::AbstractStateGenerator;

  public:
    PathGenerator(std::vector<const Generator*>& generators);

    void GeneratePath(Path& waypoint_path);

    const Indexer& GetIndexer() const { return indexer_; }

  private:
    void InitializeGenerators();

    void TopologicalSort();

    std::vector<const Generator*>* generators_;
    Indexer indexer_;
};
}  // namespace autonav

#endif  // PATH_GENERATOR_HPP_
