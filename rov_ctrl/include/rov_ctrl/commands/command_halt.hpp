#ifndef ROV_CTRL_COMMANDHALT_HPP
#define ROV_CTRL_COMMANDHALT_HPP

#include "rov_ctrl/commands/generic_command.hpp"
#include "rov_ctrl/states/state_halt.hpp"

namespace rov {

namespace commands {

    class CommandHalt : public GenericCommand {

    private:
        std::shared_ptr<states::StateHalt> stateHalt_;

    public:
        CommandHalt();
        virtual ~CommandHalt() override;
        virtual fsm::retval Execute() override;

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ROV_CTRL_COMMANDHALT_HPP
