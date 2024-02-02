#include "rov_ctrl/commands/command_surgeyawrate.hpp"

namespace rov {

namespace commands {

    CommandSurgeYawRate::CommandSurgeYawRate() {}

    CommandSurgeYawRate::~CommandSurgeYawRate() {}

    fsm::retval CommandSurgeYawRate::Execute()
    {
        return fsm_->SetNextState(rov::states::ID::velocity);
    }

    void CommandSurgeYawRate::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateSurgeYawRate_ = std::dynamic_pointer_cast<states::StateSurgeYawRate>(state);
    }

    void CommandSurgeYawRate::SetTimeout(uint timeout_sec)
    {
        stateSurgeYawRate_->timeout = timeout_sec;
        stateSurgeYawRate_->SetSurgeYawRate(0.0, 0.0);
    }
}
}
