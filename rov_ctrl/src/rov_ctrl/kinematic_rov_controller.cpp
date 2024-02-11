
#include "rov_ctrl/kinematic_rov_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jsoncpp/json/json.h>

#include "rov_ctrl/configuration.hpp"
//#include "ulisse_ctrl/states/generic_state.hpp"

//#include "ulisse_msgs/terminal_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "rov_msgs/topicnames.hpp"
#include "rov_ctrl/rov_defines.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rov {

ROVController::ROVController(std::string conf_filename)
    : Node("kinematic_control_node")
    , boundariesSet_(false)
{
    std::cout << "Welcome to ROV controller! " << std::endl;
    std::cout << std::endl;

    fileName_ = conf_filename;

    ctrlData_ = std::make_shared<ControlData>();

    stateHalt_ = std::make_shared<states::StateHalt>();
    stateHold_ = std::make_shared<states::StateHold>();
    stateLatLong_ = std::make_shared<states::StateLatLong>();
    stateSurgeYawRate_ = std::make_shared<states::StateSurgeYawRate>();

    //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr client = node->create_client<rov_msgs::srv::UserInput>("user_input");
    //cliUserInput_ = this->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);

    navFilterSub_ = this->create_subscription<rov_msgs::msg::NavFilterData>(rov_msgs::topicnames::nav_filter_data, 10, std::bind(&ROVController::NavFilterCB, this, _1));

    // Control Publisher
    genericLogPub_ = this->create_publisher<std_msgs::msg::String>("/rov/log/generic", 10);
    vehicleStatusPub_ = this->create_publisher<rov_msgs::msg::VehicleStatus>(rov_msgs::topicnames::vehicle_status, 10);
    referenceVelocitiesPub_ = this->create_publisher<rov_msgs::msg::ReferenceVelocities>(rov_msgs::topicnames::reference_velocities, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    //tpikActionPub_ = this->create_publisher<ulisse_msgs::msg::TPIKAction>(ulisse_msgs::topicnames::tpik_action, 10);

    //safetyBoundarySetPub_ = this->create_publisher<std_msgs::msg::Bool>(ulisse_msgs::topicnames::safety_boundary_set, 10);

    /// TPIK Manager
    actionManager_ = std::make_shared<tpik::ActionManager>(tpik::ActionManager());

    /// ROBOT MODEL
    Eigen::TransformationMatrix world_T_vehicle;

    /// Jacobian
    Eigen::Matrix6d J_ROV;
    J_ROV.setIdentity();

    // Robot Model
    robotModel_ = std::make_shared<rml::RobotModel>(world_T_vehicle, rov::robotModelID::blueROV, J_ROV);

    // ***** SETUP TASKS ***** //

    // ROV CONTROL VELOCITY LINEAR
    rovLinearVelocity_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(rov::task::rovLinearVelocity, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovLinearVelocity_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_linear_velocity, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovLinearVelocity, taskInfo_));


    // ROV CONTROL VELOCITY ANGULAR
    rovAngularVelocity_ = std::make_shared<ikcl::AngularVelocity>(ikcl::AngularVelocity(rov::task::rovAngularVelocity, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovAngularVelocity_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_angular_velocity, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovAngularVelocity, taskInfo_));

    // ROV CONTROL ANGULAR POSITION
    rovAngularPosition_ = std::make_shared<ikcl::AlignToTarget>(ikcl::AlignToTarget(rov::task::rovAngularPosition, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovAngularPosition_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_angular_position, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovAngularPosition, taskInfo_));

    // ASV CONTROL DISTANCE
    rovCartesianDistance_ = std::make_shared<ikcl::CartesianDistance>(ikcl::CartesianDistance(rov::task::rovCartesianDistance, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovCartesianDistance_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_cartesian_distance, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovCartesianDistance, taskInfo_));

    // ROV SAFETY BOUNDARIES (INEQUALITY TASK)
    //rovSafetyBoundaries_ = std::make_shared<ikcl::SafetyBoundaries>(ikcl::SafetyBoundaries(rov::task::rovSafetyBoundaries, robotModel_, rov::robotModelID::blueROV));
    //taskInfo_.task = rovSafetyBoundaries_;
    //taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_safety_boundaries, 1);
    //tasksMap_.insert(std::make_pair(rov::task::rovSafetyBoundaries, taskInfo_));

    // ROV absolute axis alignment task
    rovAbsoluteAxisAlignment_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(rov::task::rovAbsoluteAxisAlignment, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovAbsoluteAxisAlignment_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_absolute_axis_alignment, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovAbsoluteAxisAlignment, taskInfo_));

    // ROV absolute axis alignment task
    //rovAbsoluteAxisAlignmentSafety_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(rov::task::rovAbsoluteAxisAlignmentSafety, robotModel_, rov::robotModelID::blueROV));
    //taskInfo_.task = rovAbsoluteAxisAlignmentSafety_;
    //taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_absolute_axis_alignment_safety, 1);
    //tasksMap_.insert(std::make_pair(rov::task::rovAbsoluteAxisAlignmentSafety, taskInfo_));

    // ROV absolute axis alignment task hold
    rovAbsoluteAxisAlignmentHold_ = std::make_shared<ikcl::AbsoluteAxisAlignment>(ikcl::AbsoluteAxisAlignment(rov::task::rovAbsoluteAxisAlignmentHold, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovAbsoluteAxisAlignmentHold_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_absolute_axis_alignment_hold, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovAbsoluteAxisAlignmentHold, taskInfo_));

    // ROV CONTROL VELOCITY LINEAR HOLD
    rovLinearVelocityHold_ = std::make_shared<ikcl::LinearVelocity>(ikcl::LinearVelocity(rov::task::rovLinearVelocityHold, robotModel_, rov::robotModelID::blueROV));
    taskInfo_.task = rovLinearVelocityHold_;
    taskInfo_.taskPub = this->create_publisher<rov_msgs::msg::TaskStatus>(rov_msgs::topicnames::task_linear_velocity_hold, 1);
    tasksMap_.insert(std::make_pair(rov::task::rovLinearVelocityHold, taskInfo_));

    // Service
    //srvUserInput_ = this->create_service<rov_msgs::srv::command>(rov_msgs::topicnames::user_input_service, std::bind(&ROVController::CommandsHandler, this, _1, _2, _3));
    srvControlCommand_ = this->create_service<rov_msgs::srv::ControlCommand>(rov_msgs::topicnames::control_cmd_service, std::bind(&ROVController::CommandsHandler, this, _1, _2, _3));
    srvUserInput_ = this->create_service<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service, std::bind(&ROVController::userInputHandler, this, _1, _2, _3));

    // Setup Params for Tasks and iCAT
    // Initialize solver_ and iCAT
    int dof = 6;
    iCat_ = std::make_shared<tpik::iCAT>(tpik::iCAT(dof));

    yTpik_ = Eigen::VectorXd::Zero(dof);

    // load config file
    conf_ = std::make_shared<KCLConfiguration>();
    if (!LoadConfiguration(conf_)) {
        std::cerr << "Failed to load KCL configuration from file" << std::endl;
        return;
    }
    // solver_ definition
    solver_ = std::make_shared<tpik::Solver>(tpik::Solver(actionManager_, iCat_));

    // FSM Initialization
    SetUpFSM();


    current_state = rov::states::ID::hold;
    std::cout << "initial state: " << current_state << std::endl;

    boundariesSet_ = true;

    // Main function timer
    int msRunPeriod = 1.0 / (100.0) * 1000;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&ROVController::Run, this));

    dirV.setZero(); bodyF_dirV.setZero(); reference_speed.setZero();

}

bool ROVController::LoadConfiguration(std::shared_ptr<KCLConfiguration>& conf)
{
    libconfig::Config confObj;

    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD CONFIGURATION FROM NAV FILTER TO READ CENTROID
    ///
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter_rov");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter_rov.conf");

    // Read the file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    Eigen::VectorXd centroidLocationTmp;
    if (!ctb::GetParamVector(confObj, centroidLocationTmp, "centroidLocation")) {
        std::cerr << "Failed to load centroidLocation from file" << std::endl;
        return false;
    };

    centroidLocation_.latitude = centroidLocationTmp[0];
    centroidLocation_.longitude = centroidLocationTmp[1];

    std::cout << "Centroid: " << centroidLocation_.latitude << ", " << centroidLocation_.longitude << std::endl;

    //rovSafetyBoundaries_->Centroid() = centroidLocation_;

    ///////////////////////////////////////////////////////////////////////////
    /////        LOAD KCL CONFIGURATION
    ///
    package_share_directory = ament_index_cpp::get_package_share_directory("rov_ctrl");
    confPath = package_share_directory;
    confPath.append("/conf/");
    confPath.append(fileName_);

    std::cout << "PATH TO KCL CONF FILE : " << confPath << std::endl;

    // Read the configuration file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    std::cout << "reading configuration done " << std::endl;

    if (!conf->ConfigureFromFile(confObj)) {
        std::cerr << "Failed to load KCL configuration" << std::endl;
        return false;
    }

    // Set Saturation values for the iCAT (read from conf file)
    iCat_->SetSaturation(conf->saturationMin, conf->saturationMax);

    //std::cout << tc::brown << *conf << tc::none << std::endl;

    if (!ConfigureTasksFromFile(tasksMap_, confObj)) {
        std::cerr << "Failed to load Tasks from file" << std::endl;
        return false;
    };
    std::cout<< "Tasks configured!" << std::endl << std::endl;

    if (!ConfigurePriorityLevelsFromFile(actionManager_, tasksMap_, confObj)) {
        std::cerr << "Failed to load Priority Levels from file" << std::endl;
        return false;
    };
    std::cout<< "PLs configured!" <<std::endl <<std::endl;

    if (!ConfigureActionsFromFile(actionManager_, confObj)) {
        std::cerr << "Failed to load  Actions from file" << std::endl;
        return false;
    };
    std::cout<< "Actions configured!" <<std::endl << std::endl;

    //insert states in the map

    statesMap_.insert({ rov::states::ID::halt, stateHalt_ });
    statesMap_.insert({ rov::states::ID::hold, stateHold_ });
    statesMap_.insert({ rov::states::ID::latlongalt, stateLatLong_ });
    statesMap_.insert({ rov::states::ID::velocity, stateSurgeYawRate_ });

    if (!ConfigureSatesFromFile(statesMap_, confObj)) {
        std::cerr << "Failed to load States from file" << std::endl;
        return false;
    };

    //insert command in the map
    commandsMap_.insert({ rov::commands::ID::halt, commandHalt_ });
    commandsMap_.insert({ rov::commands::ID::hold, commandHold_ });
    commandsMap_.insert({ rov::commands::ID::latlongalt, commandLatLong_ });
    commandsMap_.insert({ rov::commands::ID::velocity, commandSurgeYawRate_ });

    return true;
}

void ROVController::PublishLog(std::string log)
{
    std_msgs::msg::String genericLogPub_msg;
    genericLogPub_msg.data = log;
    genericLogPub_->publish(genericLogPub_msg);
}

void ROVController::SetUpFSM()
{

    // ***** COMMANDS ***** //
    commandHalt_.SetFSM(&uFsm_);
    commandHalt_.SetState(stateHalt_);

    commandHold_.SetFSM(&uFsm_);
    commandHold_.SetState(stateHold_);

    commandLatLong_.SetFSM(&uFsm_);
    commandLatLong_.SetState(stateLatLong_);

    commandSurgeYawRate_.SetFSM(&uFsm_);
    commandSurgeYawRate_.SetState(stateSurgeYawRate_);

    // ***** STATES ***** //
    //Set the fsm and the structure that the states need.
    for (auto& state : statesMap_) {
        state.second->actionManager = actionManager_;
        state.second->robotModel = robotModel_;
        state.second->tasksMap = tasksMap_;
        state.second->ctrlData = ctrlData_;
        state.second->SetFSM(&uFsm_);
    }

    // ***** EVENTS ***** //
    eventRcEnabled_.SetFSM(&uFsm_);
    eventNearGoalPosition_.SetFSM(&uFsm_);
    eventNearGoalPosition_.ControlData() = ctrlData_;
    eventNearGoalPosition_.GoToHoldAfterMove(conf_->goToHoldAfterMove);
    eventNearGoalPosition_.StateHold() = std::dynamic_pointer_cast<rov::states::StateHold>(statesMap_.find(rov::states::ID::hold)->second);

    // ***** FSM CONFIGURATION ***** //
    // ADD COMMANDS
    for (auto& command : commandsMap_) {
        uFsm_.AddCommand(command.first, &command.second);
    }

    // ADD STATES
    for (auto& state : statesMap_) {
        uFsm_.AddState(state.first, state.second.get());
    }

    // ADD EVENTS
    uFsm_.AddEvent(rov::events::names::rcenabled, &eventRcEnabled_);
    uFsm_.AddEvent(rov::events::names::neargoalposition, &eventNearGoalPosition_);

    // ENABLE TRANSITIONS
    for (auto& currentState : statesMap_) {

        for (auto& nextState : statesMap_) {

            if (nextState.first != currentState.first)
                uFsm_.EnableTransition(currentState.first, nextState.first, true);
        }
    }

    // ENABLE COMMANDS
    for (auto& state : statesMap_) {
        for (auto& command : commandsMap_) {
            uFsm_.EnableCommandInState(state.first, command.first, true);
        }
    }
    uFsm_.SetInitState(rov::states::ID::hold);
}


void ROVController::Run(){
    Eigen::RotationMatrix Rz, Ry, Rx;
    Rz << cos(ctrlData_->bodyF_angularPosition.Yaw()), -sin(ctrlData_->bodyF_angularPosition.Yaw()), 0,
        sin(ctrlData_->bodyF_angularPosition.Yaw()), cos(ctrlData_->bodyF_angularPosition.Yaw()), 0,
        0, 0, 1;

    Ry << cos(ctrlData_->bodyF_angularPosition.Pitch()), 0, sin(ctrlData_->bodyF_angularPosition.Pitch()),
        0, 1, 0,
        -sin(ctrlData_->bodyF_angularPosition.Pitch()), 0, cos(ctrlData_->bodyF_angularPosition.Pitch());

    Rx << 1, 0, 0,
        0, cos(ctrlData_->bodyF_angularPosition.Roll()), -sin(ctrlData_->bodyF_angularPosition.Roll()),
        0, sin(ctrlData_->bodyF_angularPosition.Roll()), cos(ctrlData_->bodyF_angularPosition.Roll());

    Eigen::RotationMatrix worldF_R_bodyF_ = Rz * Ry * Rx;
    Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF_.transpose();

    //Eigen::Vector6d dirV;
    SetDirectionVector(dirV);
    //Eigen::Vector3d bodyF_dirV;
    bodyF_dirV = bodyF_R_worldF * dirV.head(3);
    if (uFsm_.GetCurrentStateName() == rov::states::ID::velocity) {
        stateSurgeYawRate_->goalSurge = bodyF_dirV[0] * reference_speed[0];
        stateSurgeYawRate_->goalSway = bodyF_dirV[1] * reference_speed[1];
        stateSurgeYawRate_->goalHeave = bodyF_dirV[2] * reference_speed[2];
        stateSurgeYawRate_->goalRollRate = dirV[3] * reference_speed[3];
        stateSurgeYawRate_->goalPitchRate = dirV[4] * reference_speed[4];
        stateSurgeYawRate_->goalYawRate = dirV[5] * reference_speed[5];
    }

    if (boundariesSet_) {

        // Switch State (if something happens)
        uFsm_.SwitchState();
        // Process Events
        uFsm_.ProcessEventQueue();
        // Execute current state
        uFsm_.ExecuteState();

        for (auto& taskMap : tasksMap_) {
            try {
                taskMap.second.task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        // Computing Kinematic Control via TPIK
        yTpik_ = solver_->ComputeVelocities();

        //for (int i = 0; i < yTpik_.size(); i++) {
        //    if (std::isnan(yTpik_(i))) {
        //        yTpik_(i) = 0.0;
        //        RCLCPP_INFO(this->get_logger(), "NaN requested velocity");
        //    }
        //}
    }



    tNow_ = std::chrono::system_clock::now();
    PublishControl();
    PublishTasksInfo();
    PublishTF();

    //rclcpp::shutdown();
}

void ROVController::PublishControl(){

    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    // Publish vehicle status
    rov_msgs::msg::VehicleStatus vehicleStatusMsg;
    vehicleStatusMsg.stamp.sec = now_stamp_secs;
    vehicleStatusMsg.stamp.nanosec = now_stamp_nanosecs;
    //vehicleStatusMsg.vehicle_state = current_state;
    vehicleStatusMsg.vehicle_state = uFsm_.GetCurrentStateName();
    vehicleStatusPub_->publish(vehicleStatusMsg);

    referenceVelocities_.stamp.sec = now_stamp_secs;
    referenceVelocities_.stamp.nanosec = now_stamp_nanosecs;
    //referenceVelocitiesPub_->publish(referenceVelocities_);

    // Publish reference velocities, for the DCL, only if we are not in HALT state
    /**/

    // Publish reference velocities, for the DCL, only if we are not in HALT state
    if (uFsm_.GetCurrentStateName() != rov::states::ID::halt) {

        referenceVelocities_.desired_surge = yTpik_[0];
        referenceVelocities_.desired_sway = yTpik_[1];
        referenceVelocities_.desired_heave = yTpik_[2];
        referenceVelocities_.desired_roll_rate = yTpik_[3];
        referenceVelocities_.desired_pitch_rate = yTpik_[4];
        referenceVelocities_.desired_yaw_rate = yTpik_[5];

        referenceVelocitiesPub_->publish(referenceVelocities_);
        std::cout<< "Tpik =  "<< yTpik_<< std::endl;
    }

}

void ROVController::CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<rov_msgs::srv::ControlCommand::Request> request,
                                       std::shared_ptr<rov_msgs::srv::ControlCommand::Response> response)
{
    // Create a callback function for when service requests are received.

    (void)request_header;
    //RCLCPP_INFO(this->get_logger(), "Incoming request: %d", request->motion_type);
    std::stringstream logg;
    logg << "Incoming request: " << request->command_type.c_str();
    PublishLog(logg.str().c_str());
    fsm::retval ret = fsm::ok;

    //std::stringstream logg;
    //logg << "Incoming request: " << request->motion_type;

    //std::stringstream log;
    if (request->command_type == rov::commands::ID::halt) {
        std::cout << "Received Command Halt" << std::endl;
        current_state = rov::states::ID::halt;
        PublishLog("Received Command Halt");
    }
    else if (request->command_type == rov::commands::ID::hold) {
        commandHold_.SetPositionToHold(ctrlData_->inertialF_linearPosition, ctrlData_->inertialF_altitude);
        std::cout << "Received Command Hold" << std::endl;
        current_state = rov::states::ID::hold;
    }
    else if (request->command_type == rov::commands::ID::latlongalt) {
        if(!commandLatLong_.SetGoTo(LatLong(request->moveto_cmd.goal.latlong.latitude, request->moveto_cmd.goal.latlong.longitude),
                                    request->moveto_cmd.goal.altitude, request->moveto_cmd.acceptance_radius)){
            response->res = "CommandAnswer::fail - Malformed LatLong Message.";
            ret = fsm::retval::fail;
        }
        std::cout << "Received Command MoveTo" << std::endl;
        current_state = rov::states::ID::latlongalt;
    }
    else if (request->command_type == rov::commands::ID::velocity) {

        //current_state = rov::states::ID::velocity;
        reference_speed[0] = request->sh_cmd.speed[0];
        reference_speed[1] = request->sh_cmd.speed[1];
        reference_speed[2] = request->sh_cmd.speed[2];
        reference_speed[3] = request->sh_cmd.heading;
        reference_speed[4] = request->sh_cmd.heading;
        reference_speed[5] = request->sh_cmd.heading;
        stateSurgeYawRate_->goalSurge = 0.0;
        stateSurgeYawRate_->goalSway = 0.0;
        stateSurgeYawRate_->goalHeave = 0.0;
        stateSurgeYawRate_->goalRollRate = 0.0;
        stateSurgeYawRate_->goalPitchRate = 0.0;
        stateSurgeYawRate_->goalYawRate = 0.0;
        commandSurgeYawRate_.SetTimeout(request->sh_cmd.timeout.sec);
        stateSurgeYawRate_->ResetTimer();
        //log << "Received Command surgeyawrate (data read from topic)";
        //PublishLog(log.str().c_str());

        std::cout << "Received Command VelocityControl" << std::endl;
        current_state = rov::states::ID::velocity;
        /*
        referenceVelocities_.desired_surge = request->sh_cmd.speed[0];
        referenceVelocities_.desired_sway = request->sh_cmd.speed[1];
        referenceVelocities_.desired_heave = request->sh_cmd.speed[2];
        referenceVelocities_.desired_roll_rate = request->sh_cmd.heading;
        referenceVelocities_.desired_pitch_rate = request->sh_cmd.heading;
        referenceVelocities_.desired_yaw_rate = request->sh_cmd.heading;
        */
        //referenceVelocitiesPub_->publish(referenceVelocities_);
    }

    else{
        response->res = "CommandAnswer::fail - Unsupported command: " + request->command_type;
        ret = fsm::retval::fail;
    }
    //option = request->command_type;

    //response->res = "good";
    //RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->res.c_str());
    if (ret == fsm::retval::ok) {
        //task update
        for (auto& taskMap : tasksMap_) {
            try {
                taskMap.second.task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        uFsm_.ExecuteCommand(request->command_type);
        response->res = "[KCL] CommandAnswer::ok";
    }
}

void ROVController::userInputHandler(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<rov_msgs::srv::UserInput::Request> request,
                                           std::shared_ptr<rov_msgs::srv::UserInput::Response> response){
    // Create a callback function for when service requests are received.
    (void) request_header;
    RCLCPP_INFO(this->get_logger(), "Incoming request: %d", request->motion_type);
    std::stringstream logg;
    logg << "Incoming request: " << request->motion_type;

    motion_direction = request->motion_type;

    response->res = "command received";
    RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->res.c_str());
}

void ROVController::SetDirectionVector(Eigen::Vector6d &d_vect){
    d_vect.setZero();
    switch(motion_direction){
    case rov::inputs::ID::halt :{
        //vehicleStatus.vehicle_state = rov::states::ID::halt;
    } break;
    case rov::inputs::ID::hold :{
        d_vect.setZero();
    } break;
    case rov::inputs::ID::forward :{
        d_vect[0] = 1;
    } break;
    case rov::inputs::ID::backward :{
        d_vect[0] = -1;
    } break;
    case rov::inputs::ID::up :{
        d_vect[2] = -1;
    } break;
    case rov::inputs::ID::down :{
        d_vect[2] = 1;
    } break;
    case rov::inputs::ID::left :{
        d_vect[1] = -1;
    } break;
    case rov::inputs::ID::right :{
        d_vect[1] = 1;
    } break;
    case rov::inputs::ID::turn_left :{
        d_vect[5] = -1;
    } break;
    case rov::inputs::ID::turn_right :{
        d_vect[5] = 1;
    } break;
    default:{
        std::cout << "direction not supported (user input)" << std::endl;
    } break;
    }
}


ROVController::~ROVController() { }

void ROVController::NavFilterCB(const rov_msgs::msg::NavFilterData::SharedPtr msg)
{
    ctrlData_->inertialF_linearPosition.latitude = msg->inertialframe_linear_position.latlong.latitude;
    ctrlData_->inertialF_linearPosition.longitude = msg->inertialframe_linear_position.latlong.longitude;
    ctrlData_->inertialF_altitude = msg->inertialframe_linear_position.altitude;

    // Get the water current for hold state
    ctrlData_->inertialF_waterCurrent[0] = msg->inertialframe_water_current[0];
    ctrlData_->inertialF_waterCurrent[1] = msg->inertialframe_water_current[1];
    ctrlData_->inertialF_waterCurrent[2] = msg->inertialframe_water_current[2];

    ctrlData_->bodyF_angularPosition.Roll(msg->bodyframe_angular_position.roll);
    ctrlData_->bodyF_angularPosition.Pitch(msg->bodyframe_angular_position.pitch);
    ctrlData_->bodyF_angularPosition.Yaw(msg->bodyframe_angular_position.yaw);

    ctrlData_->bodyF_linearVelocity[0] = msg->bodyframe_linear_velocity[0];
    ctrlData_->bodyF_linearVelocity[1] = msg->bodyframe_linear_velocity[1];
    ctrlData_->bodyF_linearVelocity[2] = msg->bodyframe_linear_velocity[2];

    ctrlData_->bodyF_angularVelocity[0] = msg->bodyframe_angular_velocity[0];
    ctrlData_->bodyF_angularVelocity[1] = msg->bodyframe_angular_velocity[1];
    ctrlData_->bodyF_angularVelocity[2] = msg->bodyframe_angular_velocity[2];

    // Linear position in world frame
    Eigen::Vector3d worldF_vehicleLinearPosition(ctrlData_->inertialF_linearPosition.latitude, ctrlData_->inertialF_linearPosition.longitude, ctrlData_->inertialF_altitude);

    // Updating the robot model
    Eigen::TransformationMatrix worldF_T_vehicleF;
    worldF_T_vehicleF.TranslationVector(worldF_vehicleLinearPosition);
    worldF_T_vehicleF.RotationMatrix(ctrlData_->bodyF_angularPosition.ToRotationMatrix());

    Eigen::Vector6d velocity_fbk = Eigen::Vector6d::Zero();
    velocity_fbk(0) = ctrlData_->bodyF_linearVelocity[0];
    velocity_fbk(1) = ctrlData_->bodyF_linearVelocity[1];
    velocity_fbk(2) = ctrlData_->bodyF_linearVelocity[2]; // what about the angular velocity???

    robotModel_->PositionOnInertialFrame(worldF_T_vehicleF);
    robotModel_->VelocityVector(rov::robotModelID::blueROV, velocity_fbk);
}

void ROVController::PublishTF(){
    // Publish simplified data for the ROV Rviz visualization
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
    t_stamp_goals.header.stamp.sec = now_stamp_secs;
    t_stamp_goals.header.stamp.nanosec = now_stamp_nanosecs;

    //t_stamp_goals.header.stamp = this->get_clock()->now();
    t_stamp_goals.header.frame_id = "world";
    t_stamp_goals.child_frame_id = "GOAL";

    if (uFsm_.GetCurrentStateName() == rov::states::ID::hold) {
        Eigen::Vector3d goal_pos;
        ctb::LatLong2LocalUTM(stateHold_->positionToHold, stateHold_->altitudeToHold, centroidLocation_, goal_pos);
        t_stamp_goals.transform.translation.x = goal_pos.x();
        t_stamp_goals.transform.translation.y = goal_pos.y();
        t_stamp_goals.transform.translation.z = goal_pos.z();
        t_stamp_goals.transform.rotation.x = 1.0;
        t_stamp_goals.transform.rotation.y = 0.0;
        t_stamp_goals.transform.rotation.z = 0.0;
        t_stamp_goals.transform.rotation.w = 0.0;

    } else if (uFsm_.GetCurrentStateName() == rov::states::ID::latlongalt) {
        Eigen::Vector3d goal_pos;
        ctb::LatLong2LocalUTM(stateLatLong_->goalPosition, stateLatLong_->goalAltitude, centroidLocation_, goal_pos);
        t_stamp_goals.transform.translation.x = goal_pos.x();
        t_stamp_goals.transform.translation.y = goal_pos.y();
        t_stamp_goals.transform.translation.z = goal_pos.z();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, stateLatLong_->goalHeading);
        t_stamp_goals.transform.rotation.x = q.x();
        t_stamp_goals.transform.rotation.y = q.y();
        t_stamp_goals.transform.rotation.z = q.z();
        t_stamp_goals.transform.rotation.w = q.w();

    }
    tf_broadcaster_->sendTransform(t_stamp_goals);
}

void ROVController::PublishTasksInfo()
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    // Publish Tasks Information
    for (auto& taskMap : tasksMap_) {
        try {

            std::vector<double> diagonal_internal_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->InternalActivationFunction().rows(); i++) {
                diagonal_internal_activation_function.push_back(taskMap.second.task->InternalActivationFunction().at(i, i));
            }

            std::vector<double> diagonal_external_activation_function;
            for (unsigned int i = 0; i < taskMap.second.task->ExternalActivationFunction().rows(); i++) {
                diagonal_external_activation_function.push_back(taskMap.second.task->ExternalActivationFunction().at(i, i));
            }

            std::vector<double> referenceRate;
            for (unsigned int i = 0; i < taskMap.second.task->ReferenceRate().size(); i++) {
                referenceRate.push_back(taskMap.second.task->ReferenceRate().at(i));
            }

            rov_msgs::msg::TaskStatus taskstatus_msg;

            taskstatus_msg.stamp.sec = now_stamp_secs;
            taskstatus_msg.stamp.nanosec = now_stamp_nanosecs;
            taskstatus_msg.id = taskMap.second.task->ID();
            taskstatus_msg.in_current_action = actionManager_->IsTaskInCurrentAction(taskMap.second.task->ID());
            taskstatus_msg.enabled = taskMap.second.task->Enabled();
            taskstatus_msg.external_activation_function = diagonal_external_activation_function;
            taskstatus_msg.internal_activation_function = diagonal_internal_activation_function;
            taskstatus_msg.reference_rate = referenceRate;

            tasksMap_[taskMap.second.task->ID()].taskPub->publish(taskstatus_msg);

        } catch (tpik::ExceptionWithHow& e) {
            std::cerr << "LOG TASK EXCEPTION" << std::endl;
            std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
        }
    }   
}


} // namespace rov
