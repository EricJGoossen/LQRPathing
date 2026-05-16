#ifndef VELOCITY_STATE_GENERATOR_HPP_
#define VELOCITY_STATE_GENERATOR_HPP_

#include "state_generators/abstract_state_generator.hpp"
#include "state_generators/position_state_generator.hpp"

namespace autonav::state_generators {
class VelocityStateGenerator : public AbstractStateGenerator {
  public:
    enum State : std::uint8_t { kXDot = 0, kYDot = 1, kThetaDot = 2, kDimensions = 3 };

    VelocityStateGenerator(const VelocityStateGenerator&) = delete;
    VelocityStateGenerator& operator=(const VelocityStateGenerator&) = delete;
    VelocityStateGenerator(VelocityStateGenerator&&) = default;
    VelocityStateGenerator& operator=(VelocityStateGenerator&&) = default;
    constexpr virtual ~VelocityStateGenerator() override {}
    
    std::vector<int> Dependencies() const override { return dependencies_; }
    int Id() const override { return kId; }
    int Dimension() const override { return State::kDimensions; }

    void Generate(Path& current_path)
        const override;  // TODO: Implement this function to generate the
                         // position states in the current path

    static constexpr int kId = 2;
  private:
    std::vector<int> dependencies_ = {PositionStateGenerator::kId};
};
}  // namespace autonav::state_generators

#endif  // VELOCITY_STATE_GENERATOR_HPP_