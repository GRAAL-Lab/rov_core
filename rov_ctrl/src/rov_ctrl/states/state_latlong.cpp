#include "rov_ctrl/states/state_latlong.hpp"

#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    StateLatLong::StateLatLong()
    {
        maxHeadingError_ = M_PI / 16;
        minHeadingError_ = M_PI / 32;
    }

    StateLatLong::~StateLatLong() { }

    bool StateLatLong::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(rov::states::ID::latlongalt);

        if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
            return false;
        if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
            return false;
        if (!ctb::GetParam(state, acceptanceRadius, "acceptanceRadius"))
            return false;
        return true;
    }

    fsm::retval StateLatLong::OnEntry()
    {
        //set tasks
        //safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(rov::task::rovSafetyBoundaries)->second.task);
        //absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::rovAbsoluteAxisAlignmentSafety)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::rovAbsoluteAxisAlignmentSafety)->second.task);
        cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(rov::task::rovCartesianDistance)->second.task);
        alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(rov::task::rovAngularPosition)->second.task);

        if (actionManager->SetAction(rov::action::goTo, true)) {
            return fsm::ok;
        } else {
            return fsm::fail;
        }

        /**
         * Sottoscrizione ai topic ostacolo
         */

    }

    // Callback ostacoli

    fsm::retval StateLatLong::Execute()
    {

        CheckRadioController();

        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        //safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;

        //Eigen::MatrixXd Aexternal;

        //Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());

        //absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        //absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), rov::robotModelID::blueROV);
        //absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
        //    rml::FrameID::WorldFrame);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        //double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();

        //compute the gain of the cartesian distance
        //double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        //safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());




        //goto task
        /** if (we have obstacles){
         *
         *
         *      pathController.computePath(ctrlData->inertialF_linearPosition, posizioni ostacoli, polyline)
         *
         *
         *
         *      ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, polyine(1), goalDistance, goalHeading);
         * } else {
         */

        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, goalPosition, goalDistance, goalHeading);
        double goalDistance_x, goalDistance_y, goalDistance_z;
        goalDistance_x = goalDistance * cos(goalHeading);
        goalDistance_y = goalDistance * sin(goalHeading);
        goalDistance_z = goalAltitude - ctrlData->inertialF_altitude;
        goalDistance = sqrt (pow(goalDistance,2) + pow(goalDistance_z,2));
        // }
        //std::cout << "goalDistance: "<<goalDistance << "acceptanceRadius: "<<acceptanceRadius<< "maxHeadingError_: "<<maxHeadingError_<< std::endl;
        //double finalGoalDistance, finalGoalHeading;
        //ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, goalPosition, finalGoalDistance, finalGoalHeading);
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(0, 0, 1),rml::FrameID::WorldFrame);
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(0, 0, 1), rov::robotModelID::blueROV);
        //double absoluteAxisAlignmentGain = rml::IncreasingBellShapedFunction(minWaterCurrent_, maxWaterCurrent_, 0, 1, (ctrlData->inertialF_waterCurrent).norm());
        absoluteAxisAlignmentTask_->ExternalActivationFunction() = 1.0 * Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());
        absoluteAxisAlignmentTask_->Update();


        if (goalDistance < acceptanceRadius) {
            std::cout << "*** GOAL REACHED! ***" << std::endl;
            fsm_->EmitEvent(rov::events::names::neargoalposition, rov::events::priority::medium);
            cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
        } else {

            //Set the distance vector to the target
            cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance_x, goalDistance_y, goalDistance_z), rml::FrameID::WorldFrame);
            //Set the align vector to the target
            alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance_x, goalDistance_y, 0), rml::FrameID::WorldFrame);

            //Set the vector that has to been align to the distance vector
            alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), rov::robotModelID::blueROV);



            //To avoid the case in which the error between the goal heading and the current heading is too big
            //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

            //compute the heading error
            double headingError = alignToTargetTask_->ControlVariable().norm();

            //compute the gain of the cartesian distance
            double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

            //Set the gain of the cartesian distance task
            cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
            cartesianDistanceTask_->ExternalActivationFunction()(cartesianDistanceTask_->TaskSpace()-1, cartesianDistanceTask_->TaskSpace()-1) = 1.0;
            //std::cout << "cartesianDistanceTask_->ExternalActivationFunction: " << cartesianDistanceTask_->ExternalActivationFunction();
        }

        //std::cout << "STATE LATLONG" << std::endl;

        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
