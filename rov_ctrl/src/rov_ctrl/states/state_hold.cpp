#include "rov_ctrl/states/state_hold.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    StateHold::StateHold()
        : hysteresisState_ { HysteresisState::Align }
        , goalHeading { 0.0 }
        , goalDistance { 0.0 }
        , altitudeToHold { 0.0 }
    {
    }

    StateHold::~StateHold() { }

    fsm::retval StateHold::OnEntry()
    {
        //inertialF_waterCurrent = std::make_shared<Eigen::Vector2d>();

        // Set tasks
        //safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(rov::task::rovSafetyBoundaries)->second.task);
        //absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::rovAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(rov::task::rovLinearVelocityHold)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::rovAbsoluteAxisAlignmentSafety)->second.task);
        cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(rov::task::rovCartesianDistanceHold)->second.task);

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
        if (!ctb::GetParam(state, maxSurgeComeback2Hold_, "maxSurgeComeback2HoldAcceptanceRadius"))
            return false;

        return true;
    }

    fsm::retval StateHold::Execute()
    {
        CheckRadioController();

        //hold task
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, positionToHold, goalDistance, goalHeading); //compute the distanza between the current position and the position to hold
        double goalDistance_x, goalDistance_y, goalDistance_z;
        goalDistance_x = goalDistance * cos(goalHeading);
        goalDistance_y = goalDistance * sin(goalHeading);
        goalDistance_z = altitudeToHold - ctrlData->inertialF_altitude;
        goalDistance = sqrt (pow(goalDistance,2) + pow(goalDistance_z,2));
        //std::cout<< "positionToHold = " << positionToHold <<" altitude = " << altitudeToHold<< std::endl;

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
            linearVelocityTask_->Update();

            absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(0, 0, 1),rml::FrameID::WorldFrame);
            absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(0, 0, 1), rov::robotModelID::blueROV);

            // Avoid that the roboto try to align with very small intensity of water current.
            //double absoluteAxisAlignmentGain = rml::IncreasingBellShapedFunction(minWaterCurrent_, maxWaterCurrent_, 0, 1, (ctrlData->inertialF_waterCurrent).norm());
            double absoluteAxisAlignmentGain =1;
            absoluteAxisAlignmentTask_->ExternalActivationFunction() = absoluteAxisAlignmentGain * Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());
            absoluteAxisAlignmentTask_->Update();
            cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());

        } else if (hysteresisState_ == HysteresisState::ComeBack) {

            //Set the distance vector to the target
            cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance_x, goalDistance_y, goalDistance_z), rml::FrameID::WorldFrame);

            // Slow-down and turn: compute the gain to modify the exernal activation function of linear velocity task.
            //double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1, absoluteAxisAlignmentTask_->ControlVariable().norm());
            //linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());

            double taskGain = 1;
            //Set the gain of the cartesian distance task
            cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
            linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());

        }
        //std::cout << "STATE HOLD" << std::endl;
        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
        linearVelocityTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());

        return fsm::ok;
    }
}
}
