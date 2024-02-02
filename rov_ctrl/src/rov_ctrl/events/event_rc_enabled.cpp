#include <iostream>

#include "rov_ctrl/events/event_rc_enabled.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace events {

    fsm::retval EventRCEnabled::Execute()
    {
        std::cout << "RC Controller Detected: Halting!" << std::endl;
        fsm_->SetNextState(rov::states::ID::halt);

        return fsm::retval::ok;
    }

    fsm::retval EventRCEnabled::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
