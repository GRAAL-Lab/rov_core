#ifndef ROV_CTRL_STATESURGEYAWRATE_HPP
#define ROV_CTRL_STATESURGEYAWRATE_HPP

#include "rov_ctrl/states/generic_state.hpp"

namespace rov {

namespace states {

    class StateSurgeYawRate : public GenericState {
    protected:
        std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
        std::chrono::system_clock::time_point tStart_, tNow_;
        std::chrono::seconds totalElapsed_;

        double maxYawRateError_, minYawRateError_;

    public:
        double goalSurge, goalSway, goalHeave, goalRollRate, goalPitchRate, goalYawRate, timeout;

        StateSurgeYawRate();
        ~StateSurgeYawRate() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        void ResetTimer();

        // not needed
        void SetSurgeYawRate(double surge, double sway, double heave, double rollrate, double pitchrate, double yawrate);
        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
} // namespace states
} // namespace ulisse

#endif // ROV_CTRL_STATESURGEYAWRATE_HPP
