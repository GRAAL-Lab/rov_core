#include "rov_ctrl/states/state_surgeyawrate.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    StateSurgeYawRate::StateSurgeYawRate() : goalSurge(0.0), goalSway(0.0), goalHeave(0.0), goalRollRate(0.0), goalPitchRate(0.0), goalYawRate(0.0)
    {
    }

    StateSurgeYawRate::~StateSurgeYawRate() { }

    void StateSurgeYawRate::ResetTimer()
    {
        tStart_ = std::chrono::system_clock::now();
    }

    void StateSurgeYawRate::SetSurgeYawRate(double surge, double sway, double heave, double rollrate, double pitchrate, double yawrate)
    {
        goalSurge = surge;
        goalSway = sway;
        goalHeave = heave;
        goalRollRate = rollrate;
        goalPitchRate = pitchrate;
        goalYawRate = yawrate;
    }

    bool StateSurgeYawRate::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        (void) confObj;
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(rov::states::ID::velocity);

        if (!ctb::GetParam(state, maxYawRateError_, "maxYawRateError"))
            return false;
        if (!ctb::GetParam(state, minYawRateError_, "minYawRateError"))
            return false;
        if (!ctb::GetParam(state, minZalignmentError_, "minZalignmentError"))
            return false;
        if (!ctb::GetParam(state, maxZalignmentError_, "maxZalignmentError"))
            return false;

        return true;
    }

    fsm::retval StateSurgeYawRate::OnEntry()
    {
        // Set tasks
        /*safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        */
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::rovAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(rov::task::rovLinearVelocity)->second.task);
        angularVelocityTask_ = std::dynamic_pointer_cast<ikcl::AngularVelocity>(tasksMap.find(rov::task::rovAngularVelocity)->second.task);

        if (actionManager->SetAction(rov::action::velocity, true)) {
            return fsm::ok;
        } else {
            return fsm::fail;
        }
    }

    fsm::retval StateSurgeYawRate::Execute()
    {
        CheckRadioController();

        tNow_ = std::chrono::system_clock::now();
        totalElapsed_ = std::chrono::duration_cast<std::chrono::seconds>(tNow_ - tStart_);


        // Set a velocity to point to the circle in case of the catamaran  slips away.
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(0, 0, 1),rml::FrameID::WorldFrame);
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(0, 0, 1), rov::robotModelID::blueROV);
        absoluteAxisAlignmentTask_->ExternalActivationFunction() = 1.0 * Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());
        absoluteAxisAlignmentTask_->Update();

        linearVelocityTask_->SetReferenceRate(Eigen::Vector3d(goalSurge, goalSway, goalHeave), robotModel->BodyFrameID());
        linearVelocityTask_->Update();
        // Slow-down and turn: compute the gain to modify the exernal activation function of linear velocity task.
        double taskGain = 1;
        //linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());

        angularVelocityTask_->SetReferenceRate(Eigen::Vector3d(goalRollRate, goalPitchRate, goalYawRate), robotModel->BodyFrameID());
        //angularVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(angularVelocityTask_->TaskSpace(), angularVelocityTask_->TaskSpace());

        double alignmentError = absoluteAxisAlignmentTask_->ControlVariable().norm();
        double alignGain = rml::DecreasingBellShapedFunction(minZalignmentError_, maxZalignmentError_, 0, 1.0, alignmentError);
        linearVelocityTask_->ExternalActivationFunction() = alignGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());
        angularVelocityTask_->ExternalActivationFunction() = alignGain * Eigen::MatrixXd::Identity(angularVelocityTask_->TaskSpace(), angularVelocityTask_->TaskSpace());

        return fsm::ok;
    }

    fsm::retval StateSurgeYawRate::OnExit(){
        linearVelocityTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());
        angularVelocityTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(angularVelocityTask_->TaskSpace(), angularVelocityTask_->TaskSpace());

        return fsm::ok;
    }

} // namespace states
} // namespace rov



