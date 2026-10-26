// Host stand-in: the controller accepts a null cap bank, which is what the comparison passes.
#ifndef CAP_BANK_SUBSYSTEM_HPP_
#define CAP_BANK_SUBSYSTEM_HPP_

namespace aruwsrc::communication::can::cap_bank
{
enum class SprintMode
{
    NO_SPRINT,
    SPRINT,
};
}  // namespace aruwsrc::communication::can::cap_bank

namespace aruwsrc::control::cap_bank
{
class CapBankSubsystem
{
public:
    float getAvailableEnergy() const { return 0; }
    void changeSprintMode(aruwsrc::communication::can::cap_bank::SprintMode) {}
};
}  // namespace aruwsrc::control::cap_bank

#endif  // CAP_BANK_SUBSYSTEM_HPP_
