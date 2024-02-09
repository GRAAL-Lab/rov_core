#ifndef ROV_CTRL_STATEHOLD_HPP
#define ROV_CTRL_STATEHOLD_HPP

#include "rov_ctrl/states/generic_state.hpp"
#include "rov_ctrl/states/state_latlong.hpp"

namespace rov {

namespace states {

    enum class HysteresisState{
        Align,
        ComeBack
    };

    /**
     * @brief The StateHold class
     */
    class StateHold : public GenericState {

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
        double minWaterCurrent_, maxWaterCurrent_;
        double maxSurgeComeback2HoldAcceptanceRadius_;
        HysteresisState hysteresisState_;

    public:
        StateHold();
        ~StateHold() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        //std::shared_ptr<Eigen::Vector2d> inertialF_waterCurrent;

        double maxAcceptanceRadius;
        double minAcceptanceRadius;
        double goalHeading;
        double goalDistance;

        ctb::LatLong positionToHold;
        double altitudeToHold;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ROV_CTRL_STATEHOLD_HPP
