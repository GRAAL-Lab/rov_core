#include "rov_ctrl/states/state_hold.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    StateHold::StateHold()
        : hysteresisState_ { HysteresisState::Align }
        , goalDistance { 0.0 }
    {
    }

    StateHold::~StateHold() { }

    fsm::retval StateHold::OnEntry()
    {
        //inertialF_waterCurrent = std::make_shared<Eigen::Vector2d>();

        // Set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(rov::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(rov::task::asvLinearVelocityHold)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::asvAbsoluteAxisAlignmentHold)->second.task);

        // Set action
        if (actionManager->SetAction(rov::action::hold, true)) {
            return fsm::ok;
        } else {
            return fsm::fail;
        }
    }

    bool StateHold::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(rov::states::ID::hold);

        if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
            return false;
        if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
            return false;
        if (!ctb::GetParam(state, maxAcceptanceRadius, "maxAcceptanceRadius"))
            return false;
        if (!ctb::GetParam(state, minAcceptanceRadius, "minAcceptanceRadius"))
            return false;
        if (!ctb::GetParam(state, minWaterCurrent_, "minWaterCurrent"))
            return false;
        if (!ctb::GetParam(state, maxWaterCurrent_, "maxWaterCurrent"))
            return false;
        if (!ctb::GetParam(state, maxSurgeComeback2HoldAcceptanceRadius_, "maxSurgeComeback2HoldAcceptanceRadius"))
            return false;

        return true;
    }

    fsm::retval StateHold::Execute()
    {
        CheckRadioController();

        safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;

        Eigen::MatrixXd Aexternal;

        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());

        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), rov::robotModelID::blueROV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
            rml::FrameID::WorldFrame);

        // To avoid the case in which the error between the goal heading and the current heading is too big
        // we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        // Compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();

        // Compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // CHECK TODO: Change this function to modify Reference gain (NOT Activation GAIN)
        //safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->TaskParameter().gain = taskGainSafety * safetyBoundariesTask_->TaskParameter().conf_gain;

        /*std::cout << "minHeadingError_: " << minHeadingError_ << std::endl;
        std::cout << "maxHeadingError_: " << maxHeadingError_ << std::endl;
        std::cout << "headingErrorsafety: " << absoluteAxisAlignmentSafetyTask_->ControlVariable().norm() << std::endl;
        std::cout << "taskGainSafety: " << taskGainSafety << std::endl;
        std::cout << "-----------------------" << std::endl;*/

        //hold task
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, positionToHold, goalDistance, goalHeading); //compute the distanza between the current position and the position to hold

        // If the robot is inside the circle put the catamaran countercurrent, otherwise
        // point to the hold circle defined by maxAcceptanceRadius and minAcceptanceRadiuos
        if (goalDistance < minAcceptanceRadius) {
            hysteresisState_ = HysteresisState::Align;
        } else if (goalDistance > maxAcceptanceRadius) {
            hysteresisState_ = HysteresisState::ComeBack;
        }

        //If the goal distance is minAcceptanceRadius << goalDistance << maxAcceptanceRadius.
        if (hysteresisState_ == HysteresisState::Align) {
            linearVelocityTask_->SetReferenceRate(Eigen::Vector3d::Zero(), robotModel->BodyFrameID());

            absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(-(ctrlData->inertialF_waterCurrent).normalized().x(),
                                                                  -(ctrlData->inertialF_waterCurrent).normalized().y(), 0),
                rml::FrameID::WorldFrame);
            absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), rov::robotModelID::blueROV);

            // Avoid that the roboto try to align with very small intensity of water current.
            double absoluteAxisAlignmentGain = rml::IncreasingBellShapedFunction(minWaterCurrent_, maxWaterCurrent_, 0, 1, (ctrlData->inertialF_waterCurrent).norm());
            absoluteAxisAlignmentTask_->ExternalActivationFunction() = absoluteAxisAlignmentGain * Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());

        } else if (hysteresisState_ == HysteresisState::ComeBack) {
            // If the previos action was comeback to the hold acceptance radius, keep do it until d < minAcceptanceRadius.
            absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(goalHeading), sin(goalHeading), 0.0), rml::FrameID::WorldFrame);
            double surgeReference = rml::IncreasingBellShapedFunction(minAcceptanceRadius, maxAcceptanceRadius, 0.25, maxSurgeComeback2HoldAcceptanceRadius_, goalDistance);
            absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), rov::robotModelID::blueROV);
            absoluteAxisAlignmentTask_->Update();

            // Set a velocity to point to the circle in case of the catamaran  slips away.
            linearVelocityTask_->SetReferenceRate(Eigen::Vector3d(surgeReference, 0, 0), robotModel->BodyFrameID());

            // Slow-down and turn: compute the gain to modify the exernal activation function of linear velocity task.
            double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1, absoluteAxisAlignmentTask_->ControlVariable().norm());
            linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());
        }
        //std::cout << "STATE HOLD" << std::endl;
        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        return fsm::ok;
    }
}
}
