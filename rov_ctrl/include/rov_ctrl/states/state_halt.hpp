#ifndef ROV_CTRL_STATEHALT_HPP
#define ROV_CTRL_STATEHALT_HPP

#include "rov_ctrl/states/generic_state.hpp"

namespace rov {

namespace states {

    class StateHalt : public GenericState {

    public:
        StateHalt();
        ~StateHalt() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ROV_CTRL_STATEHALT_HPP
