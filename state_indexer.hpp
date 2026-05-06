#ifndef STATE_INDEXER_HPP_
#define STATE_INDEXER_HPP_

#include <typeindex>
#include <unordered_map>
#include <stdexcept>

namespace auto_nav {

class StateIndexer {
    using StateMapping = std::unordered_map<std::type_index, uint8_t>;

  public:
    StateIndexer(StateMapping state_mapping) : state_mapping_(std::move(state_mapping)) {}

    template <typename StateGenerator>
    const uint8_t GetState(uint8_t state_offset) const {
        static const std::type_index kKey(typeid(StateGenerator));
        auto it = state_mapping_.find(kKey);

        if (it == state_mapping_.end()) {
            throw std::runtime_error("StateGenerator not found in StateMapping");
        }
        return it->second + state_offset;
    }

  private:
    StateMapping state_mapping_;
};
} // namespace auto_nav


#endif // STATE_INDEXER_HPP_