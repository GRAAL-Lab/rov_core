#ifndef ROV_CTRL_STATEMOVE_HPP
#define ROV_CTRL_STATEMOVE_HPP

#include "rov_ctrl/states/generic_state.hpp"
//#include "ulisse_ctrl/states/state_hold.hpp"

namespace rov {

namespace states {

    class StateLatLong : public GenericState {

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;

    public:
        StateLatLong();
        ~StateLatLong() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        LatLong goalPosition;
        double goalAltitude;
        double goalHeading;
        double goalDistance;
        double acceptanceRadius;

        // Classe PathController

        // Variabili ostacoli
        // Variabili polyline

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ROV_CTRL_STATEMOVE_HPP
