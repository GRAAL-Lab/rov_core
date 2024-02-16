

#include "rov_ctrl/events/event_near_goal_position.hpp"

namespace rov {

namespace events {

    fsm::retval EventNearGoalPosition::Execute()
    {
        std::cout << "Executing: EventNearGoalPosition" << std::endl;

        if (goToHold_) {
            stateHold_->positionToHold = ctrlData_->inertialF_linearPosition;
            stateHold_->altitudeToHold = ctrlData_->inertialF_altitude;
            fsm_->SetNextState("Hold");
        } else {
            fsm_->SetNextState("Halt");
        }

        return fsm::retval::ok;
    }

    fsm::retval EventNearGoalPosition::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
