#ifndef ROV_CTRL_COMMANDHOLD_HPP
#define ROV_CTRL_COMMANDHOLD_HPP

#include "rov_ctrl/commands/generic_command.hpp"
#include "rov_ctrl/states/state_hold.hpp"
#include "rov_ctrl/states/state_latlong.hpp"

namespace rov {

namespace commands {

    class CommandHold : public GenericCommand {

        std::shared_ptr<states::StateHold> stateHold_;

    public:
        CommandHold();
        virtual ~CommandHold() override;
        virtual fsm::retval Execute(void) override;

        void SetState(std::shared_ptr<states::GenericState> state) override;

        //void SetWaterCurrent(const std::shared_ptr<Eigen::Vector2d>& inertialF_waterCurrent);

        void SetPositionToHold(const ctb::LatLong& p);
    };
}
}
#endif // ROV_CTRL_COMMANDHOLD_HPP
