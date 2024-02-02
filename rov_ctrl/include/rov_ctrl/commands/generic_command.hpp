#ifndef ROV_CTRL_GENERICCOMMAND_HPP
#define ROV_CTRL_GENERICCOMMAND_HPP

#include "rov_ctrl/ctrl_data_structs.hpp"
#include "rov_ctrl/rov_defines.hpp"
#include "rov_ctrl/states/generic_state.hpp"
#include <fsm/fsm.h>

namespace rov {

namespace commands {

    class GenericCommand : public fsm::BaseCommand {

    public:
        GenericCommand();
        virtual ~GenericCommand();

        virtual void SetState(std::shared_ptr<states::GenericState> state) = 0;
    };
}
}
#endif // ROV_CTRL_GENERICCOMMAND_HPP
