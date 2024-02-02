#include "rov_ctrl/commands/command_hold.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace commands {

    CommandHold::CommandHold() { }

    CommandHold::~CommandHold() { }

    fsm::retval CommandHold::Execute()
    {
        return fsm_->SetNextState(rov::states::ID::hold);
    }

    void CommandHold::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateHold_ = std::dynamic_pointer_cast<states::StateHold>(state);
    }

    /*void CommandHold::SetWaterCurrent(const std::shared_ptr<Eigen::Vector2d>& inertialF_waterCurrent)
    {
        stateHold_->inertialF_waterCurrent = inertialF_waterCurrent;
    }*/

    void CommandHold::SetPositionToHold(const LatLong& p)
    {
        stateHold_->positionToHold = p;
    }
}
}
