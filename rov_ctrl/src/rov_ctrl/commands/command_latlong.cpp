#include "rov_ctrl/commands/command_latlong.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace commands {

    CommandLatLong::CommandLatLong() {}

    CommandLatLong::~CommandLatLong() {}

    fsm::retval CommandLatLong::Execute()
    {
        return fsm_->SetNextState(rov::states::ID::latlongalt);
    }

    bool CommandLatLong::SetGoTo(LatLong goalPosition, double acceptanceRadius)
    {
        if (std::isnan(goalPosition.latitude) ||
            std::isnan(goalPosition.longitude) ||
            std::isnan(acceptanceRadius)) {
            return false;
        } else {
            stateLatLong_->goalPosition = goalPosition;
            stateLatLong_->acceptanceRadius = acceptanceRadius;
            return true;
        }
    }

    void CommandLatLong::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateLatLong_ = std::dynamic_pointer_cast<states::StateLatLong>(state);
    }
}
}
