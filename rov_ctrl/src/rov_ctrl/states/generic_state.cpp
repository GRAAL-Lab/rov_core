#include "rov_ctrl/states/generic_state.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    GenericState::GenericState()
    {
    }

    GenericState::~GenericState()
    {
    }

    void GenericState::CheckRadioController()
    {
        if (ctrlData->radioControllerEnabled) {
            fsm_->EmitEvent(rov::events::names::rcenabled, rov::events::priority::high);
        }
    }
}
}
