#ifndef STATE_INDEXER_HPP_
#define STATE_INDEXER_HPP_

#include <cassert>
#include <cstdint>
#include <unordered_map>

#include "state_generators/abstract_state_generator.hpp"

namespace autonav {

// Where each generator's states sit in a waypoint's state vector. Generators
// are registered in dependency order, each taking the next Dimension() slots.
class Indexer {
    using Generator = state_generators::AbstractStateGenerator;

  public:
    uint8_t GetIndex(const Generator& generator, uint8_t state_offset) const {
        return GetIndex(generator.Id(), state_offset);
    }

    uint8_t GetIndex(int generator_id, uint8_t state_offset) const {
        assert(indices_.count(generator_id) && "Generator not registered");
        return indices_.at(generator_id) + state_offset;
    }

    void RegisterGenerator(const Generator& generator) {
        indices_[generator.Id()] = current_index_;
        current_index_ += generator.Dimension();
    }

    int Dimension() const { return current_index_; }

  private:
    std::unordered_map<int, uint8_t> indices_;
    int current_index_ = 0;
};
}  // namespace autonav

#endif  // STATE_INDEXER_HPP_
