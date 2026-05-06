#ifndef ABSTRACT_STATE_GENERATOR_HPP_
#define ABSTRACT_STATE_GENERATOR_HPP_

#include "../state_indexer.hpp"
#include "../path.hpp"
namespace auto_nav::state_generators {

class AbstractStateGenerator {
  protected:
    struct AbstractStateIndexer;

  public:
    using StateType = Path::StateType;

    AbstractStateGenerator(const AbstractStateGenerator&) = delete;
    AbstractStateGenerator operator=(const AbstractStateGenerator&) = delete;
    AbstractStateGenerator(AbstractStateGenerator&&) = default;
    AbstractStateGenerator& operator=(AbstractStateGenerator&&) = default;
    virtual constexpr ~AbstractStateGenerator() {}

    virtual int Id() const = 0;
    virtual int Dimension() const;
    virtual void Generate(const StateIndexer& indexer, Path& current_path) const;
};
}  // namespace auto_nav::state_generators

#endif  // ABSTRACT_STATE_GENERATOR_HPP_