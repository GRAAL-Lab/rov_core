#include "rov_ctrl/states/state_surgeyawrate.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    StateSurgeYawRate::StateSurgeYawRate() : goalSurge(0.0), goalYawRate (0.0)
    {
    }

    StateSurgeYawRate::~StateSurgeYawRate() { }

    void StateSurgeYawRate::ResetTimer()
    {
        tStart_ = std::chrono::system_clock::now();
    }

    void StateSurgeYawRate::SetSurgeYawRate(double surge, double yawrate)
    {
        goalSurge = surge;
        goalYawRate = yawrate;
    }

    bool StateSurgeYawRate::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        (void) confObj;
        //const libconfig::Setting& root = confObj.getRoot();
/*        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::surgeyawrate);

        if (!ctb::GetParam(state, maxYawRateError_, "maxYawRateError"))
            return false;
        if (!ctb::GetParam(state, minYawRateError_, "minYawRateError"))
            return false;
*/
        return true;
    }

    fsm::retval StateSurgeYawRate::OnEntry()
    {
        // Set tasks
        /*safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
*/
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

        if (timeout != 0 && totalElapsed_.count() > timeout) {
            std::cout << "Surge/YawRate Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(rov::commands::ID::halt);
        }

        // SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        // a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        // the align behavior activated in function of the internal activation function of the safety task.

//        safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;

//        Eigen::MatrixXd Aexternal;

//        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff()* Aexternal.setIdentity(
//                        absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());
//        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

//        safetyBoundariesTask_->ExternalActivationFunction() = 1.0 * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(),
//                                                                  safetyBoundariesTask_->TaskSpace());

//        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
//        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
//                                                                                                   rml::FrameID::WorldFrame);

        // To avoid the case in which the error between the goal heading and the current heading is too big
        // we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        // Compute the heading error
//        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();

//        // Compute the gain of the safety task
//        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

//        // Set the gain of the cartesian distance task
//        safetyBoundariesTask_->TaskParameter().gain = taskGainSafety * safetyBoundariesTask_->TaskParameter().conf_gain;





        return fsm::ok;
    }
} // namespace states
} // namespace ulisse



