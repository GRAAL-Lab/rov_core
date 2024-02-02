/*
 * GenericEvent.cc
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#include "ulisse_ctrl/events/generic_event.hpp"

namespace ulisse {

namespace events {

    //uint16_t GenericEvent::count_ = 0;

    GenericEvent::GenericEvent()
    {
        //context_ = NULL;
    }

    GenericEvent::~GenericEvent()
    {
        //context_ = NULL;
    }

    fsm::retval GenericEvent::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
