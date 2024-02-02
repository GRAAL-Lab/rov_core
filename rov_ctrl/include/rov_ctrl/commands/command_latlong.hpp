#ifndef ROV_CTRL_COMMANDMOVE_HPP
#define ROV_CTRL_COMMANDMOVE_HPP

#include "rov_ctrl/commands/generic_command.hpp"
#include "rov_ctrl/states/state_latlong.hpp"

namespace rov {

namespace commands {

    class CommandLatLong : public GenericCommand {

    protected:
        std::shared_ptr<states::StateLatLong> stateLatLong_;

    public:
        CommandLatLong();
        virtual ~CommandLatLong() override;
        virtual fsm::retval Execute(void) override;
        bool SetGoTo(LatLong goalPosition, double acceptanceRadius);

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ROV_CTRL_COMMANDMOVE_HPP
