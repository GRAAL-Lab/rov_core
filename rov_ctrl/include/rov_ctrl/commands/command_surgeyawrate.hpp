#ifndef ROV_CTRL_COMMANDSURGEYAWRATE_HPP
#define ROV_CTRL_COMMANDSURGEYAWRATE_HPP

#include "rov_ctrl/commands/generic_command.hpp"
#include "rov_ctrl/states/state_surgeyawrate.hpp"

namespace rov {

namespace commands {

    class CommandSurgeYawRate : public GenericCommand {

        std::shared_ptr<states::StateSurgeYawRate> stateSurgeYawRate_;

    public:
        CommandSurgeYawRate();
        virtual ~CommandSurgeYawRate() override;
        virtual fsm::retval Execute(void) override;
        void SetTimeout(uint timeout_sec);

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ROV_CTRL_COMMANDSURGEYAWRATE_HPP
