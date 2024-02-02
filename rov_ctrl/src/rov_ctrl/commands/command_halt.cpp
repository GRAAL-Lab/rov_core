#include "rov_ctrl/commands/command_halt.hpp"

namespace rov {

namespace commands {

    CommandHalt::CommandHalt() {}

    CommandHalt::~CommandHalt() {}

    fsm::retval CommandHalt::Execute()
    {
        return fsm_->SetNextState(rov::states::ID::halt);
    }

    void CommandHalt::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateHalt_ = std::dynamic_pointer_cast<states::StateHalt>(state);
    }
}
}
