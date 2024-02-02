#include "rov_ctrl/states/state_halt.hpp"
#include "rov_ctrl/rov_defines.hpp"

namespace rov {

namespace states {

    StateHalt::StateHalt() { }

    StateHalt::~StateHalt() { }

    bool StateHalt::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(rov::states::ID::halt);
        if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
            return false;
        if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
            return false;

        return true;
    }

    fsm::retval StateHalt::OnEntry()
    {
        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(rov::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(rov::task::asvAbsoluteAxisAlignmentSafety)->second.task);

        if (actionManager->SetAction(rov::action::halt, true)) {
            return fsm::ok;
        } else {
            return fsm::fail;
        }
    }

    fsm::retval StateHalt::Execute()
    {
        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;

        Eigen::MatrixXd Aexternal;

        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());
        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), rov::robotModelID::blueROV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
            rml::FrameID::WorldFrame);

           //To avoid the case in which the error between the goal heading and the current heading is too big
           //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

           //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();

           //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

           // Set the gain of the cartesian distance task
        safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

           //std::cout << "STATE HALT" << std::endl;
        return fsm::ok;
    }
}
}
