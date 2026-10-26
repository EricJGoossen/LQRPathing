#include "path_generator.hpp"

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace autonav {

PathGenerator::PathGenerator(std::vector<const Generator*>& generators)
  : generators_(&generators) {
    InitializeGenerators();
}

void PathGenerator::GeneratePath(Path& waypoint_path) {
    for (const auto& generator : *generators_) {
        generator->Generate(waypoint_path, indexer_);
    }
    waypoint_path.TimeParametrize();
}

void PathGenerator::InitializeGenerators() {
    TopologicalSort();

    for (const auto& generator : *generators_) {
        indexer_.RegisterGenerator(*generator);
    }
}

void PathGenerator::TopologicalSort() {
    std::unordered_map<int, const Generator*> GeneratorMap;
    for (const Generator* generator : *generators_) {
        GeneratorMap[generator->Id()] = generator;
    }

    std::vector<const Generator*> result;
    std::unordered_set<int> visited;
    std::unordered_set<int> inStack;
    auto dfs = [&](auto&& self, int id) -> void {
        if (visited.count(id)) {
            return;
        }

        if (inStack.count(id)) {
            throw std::runtime_error("Cycle detected in generator dependencies"
            );
        }

        inStack.insert(id);
        for (int dependency_id : GeneratorMap.at(id)->Dependencies()) {
            self(self, dependency_id);
        }
        inStack.erase(id);

        visited.insert(id);
        result.push_back(GeneratorMap.at(id));
    };

    for (const Generator* generator : *generators_) {
        dfs(dfs, generator->Id());
    }

    *generators_ = std::move(result);
}

}  // namespace autonav