#ifndef STATE_INDEXER_HPP_
#define STATE_INDEXER_HPP_

#include <cstdint>
#include <vector>
#include <cassert>

#include "state_generators/abstract_state_generator.hpp"

namespace autonav {

class Indexer {
    using Generator = state_generators::AbstractStateGenerator;

  public:
    uint8_t GetIndex(const Generator& generator, uint8_t state_offset) const {
        assert(indices_[generator.Id()] != UINT8_MAX && "Generator not registered");
        return indices_.at(generator.Id()) + state_offset;
    }

    void RegisterGenerator(const Generator& generator) {
        indices_.at(generator.Id()) = current_index_;
        current_index_ += generator.Dimension();
    }

  private:
    std::vector<uint8_t> indices_;
    int current_index_ = 0;
};
}  // namespace autonav

#endif  // STATE_INDEXER_HPP_