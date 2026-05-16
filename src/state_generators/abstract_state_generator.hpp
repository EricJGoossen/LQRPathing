#ifndef ABSTRACT_STATE_GENERATOR_HPP_
#define ABSTRACT_STATE_GENERATOR_HPP_

#include <vector>

#include "path.hpp"
namespace autonav::state_generators {

class AbstractStateGenerator {
  public:
    AbstractStateGenerator(const AbstractStateGenerator&) = delete;
    AbstractStateGenerator& operator=(const AbstractStateGenerator&) = delete;
    AbstractStateGenerator(AbstractStateGenerator&&) = default;
    AbstractStateGenerator& operator=(AbstractStateGenerator&&) = default;
    virtual constexpr ~AbstractStateGenerator() {}

    virtual std::vector<int> Dependencies() const = 0;
    virtual int Id() const = 0;
    virtual int Dimension() const = 0;

    virtual void Generate(Path& current_path)
        const = 0;
};
}  // namespace autonav::state_generators

#endif  // ABSTRACT_STATE_GENERATOR_HPP_