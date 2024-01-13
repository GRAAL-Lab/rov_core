
#include "rov_ctrl/kinematic_rov_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jsoncpp/json/json.h>

//#include "ulisse_ctrl/configuration.hpp"
//#include "ulisse_ctrl/states/generic_state.hpp"
//#include "ulisse_ctrl/ulisse_defines.hpp"

//#include "ulisse_msgs/terminal_utils.hpp"
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

    //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr client = node->create_client<rov_msgs::srv::UserInput>("user_input");
    //cliUserInput_ = this->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);

    // Control Publisher
    vehicleStatusPub_ = this->create_publisher<rov_msgs::msg::VehicleStatus>(rov_msgs::topicnames::vehicle_status, 10);
    referenceVelocitiesPub_ = this->create_publisher<rov_msgs::msg::ReferenceVelocities>(rov_msgs::topicnames::reference_velocities, 10);

    // Service
    srvUserInput_ = this->create_service<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service, std::bind(&ROVController::CommandsHandler, this, _1, _2, _3));

    // load config file
    // Setup Params for Tasks and iCAT
    if (!LoadConfiguration(conf_)) {
        std::cerr << "Failed to load KCL configuration from file" << std::endl;
        return;
    }

    current_state = rov::states::ID::hold;
    std::cout << "initial state: " << rov::states::ID::hold << std::endl;

    // Main function timer
    //int msRunPeriod = 1.0 / (conf_->controlLoopRate) * 1000;
    int msRunPeriod = 1.0 / (100.0) * 1000;
    //std::cout << " before runTimer " << std::endl;
    //std::cout << "Controller Rate: " << conf_->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&ROVController::Run, this));



}

bool ROVController::LoadConfiguration(std::shared_ptr<KCLConfiguration>& conf)
{
    libconfig::Config confObj;

    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD CONFIGURATION FROM NAV FILTER TO READ CENTROID
    ///
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter.conf");

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

    //asvSafetyBoundaries_->Centroid() = centroidLocation_;

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
    /*
    iCat_->SetSaturation(conf->saturationMin, conf->saturationMax);

    std::cout << tc::brown << *conf << tc::none << std::endl;

    if (!ConfigureTasksFromFile(tasksMap_, confObj)) {
        std::cerr << "Failed to load Tasks from file" << std::endl;
        return false;
    };
    if (!ConfigurePriorityLevelsFromFile(actionManager_, tasksMap_, confObj)) {
        std::cerr << "Failed to load Priority Levels from file" << std::endl;
        return false;
    };

    if (!ConfigureActionsFromFile(actionManager_, confObj)) {
        std::cerr << "Failed to load  Actions from file" << std::endl;
        return false;
    }; */

    //insert states in the map
    /*
    statesMap_.insert({ ulisse::states::ID::halt, stateHalt_ });
    statesMap_.insert({ ulisse::states::ID::hold, stateHold_ });
    statesMap_.insert({ ulisse::states::ID::latlong, stateLatLong_ });
    statesMap_.insert({ ulisse::states::ID::pathfollow, statePathFollowing_ });
    statesMap_.insert({ ulisse::states::ID::surgeheading, stateSurgeHeading_ });
    statesMap_.insert({ ulisse::states::ID::surgeyawrate, stateSurgeYawRate_ });

    if (!ConfigureSatesFromFile(statesMap_, confObj)) {
        std::cerr << "Failed to load States from file" << std::endl;
        return false;
    };

    //insert command in the map
    commandsMap_.insert({ ulisse::commands::ID::halt, commandHalt_ });
    commandsMap_.insert({ ulisse::commands::ID::hold, commandHold_ });
    commandsMap_.insert({ ulisse::commands::ID::latlong, commandLatLong_ });
    commandsMap_.insert({ ulisse::commands::ID::pathfollow, commandPathFollowing_ });
    commandsMap_.insert({ ulisse::commands::ID::surgeheading, commandSurgeHeading_ });
    commandsMap_.insert({ ulisse::commands::ID::surgeyawrate, commandSurgeYawRate_ });
    */


    return true;
}


void ROVController::Run(){
    //std::cout << "Forward:8, Backward:2, Left:4, Right:6, Up:9, Down:3 " << std::endl;
    //std::cout << "Insert a number to change ROV motion: " << std::endl;

    //int x;
    //std::cin >> x;
    //auto request = std::make_shared<rov_msgs::srv::UserInput::Request>();
    //std::make_shared<rov_msgs::srv::UserInput::Request> request;

    //request->motion_type = x;

    //auto result = cliUserInput_->async_send_request(request);
    //if (rclcpp::spin_until_future_complete(this, result) ==
    //    rclcpp::FutureReturnCode::SUCCESS)
    //{
    //    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "res: ", result.get()->res);
    //    std::cout << "request sent! "<< std::endl;
    //} else {
    //    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    //    std::cout << "failed.."<< std::endl;
    //}

    PublishControl();

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
    vehicleStatusMsg.vehicle_state = current_state;
    vehicleStatusPub_->publish(vehicleStatusMsg);
}

void ROVController::CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<rov_msgs::srv::UserInput::Request> request,
                                       std::shared_ptr<rov_msgs::srv::UserInput::Response> response)
{
    // Create a callback function for when service requests are received.

    (void)request_header;
    //RCLCPP_INFO(this->get_logger(), "Incoming request: %d", request->motion_type);

    //std::stringstream logg;
    //logg << "Incoming request: " << request->motion_type;

    //std::stringstream log;
    if (request->motion_type == rov::inputs::ID::halt) {
        std::cout << "Received Command Halt" << std::endl;
        current_state = rov::states::ID::halt;
    }
    else if (request->motion_type == rov::inputs::ID::hold) {
        std::cout << "Received Command Hold" << std::endl;
        current_state = rov::states::ID::hold;
    }
    else if (request->motion_type == rov::inputs::ID::forward) {
        std::cout << "Received Command Forward" << std::endl;
        current_state = rov::states::ID::forward;
    }
    else if (request->motion_type == rov::inputs::ID::backward) {
        std::cout << "Received Command Backward" << std::endl;
        current_state = rov::states::ID::backward;
    }
    else if (request->motion_type == rov::inputs::ID::right) {
        std::cout << "Received Command Right" << std::endl;
        current_state = rov::states::ID::right;
    }
    else if (request->motion_type == rov::inputs::ID::left) {
        std::cout << "Received Command Left" << std::endl;
        current_state = rov::states::ID::left;
    }
    else if (request->motion_type == rov::inputs::ID::up) {
        std::cout << "Received Command Up" << std::endl;
        current_state = rov::states::ID::up;
    }
    else if (request->motion_type == rov::inputs::ID::down) {
        std::cout << "Received Command Down" << std::endl;
        current_state = rov::states::ID::down;
    }
    else if (request->motion_type == rov::inputs::ID::turn_left) {
        std::cout << "Received Command Turn Left" << std::endl;
        current_state = rov::states::ID::turn_left;
    }
    else if (request->motion_type == rov::inputs::ID::turn_right) {
        std::cout << "Received Command Turn Right" << std::endl;
        current_state = rov::states::ID::turn_right;
    }
    else{
        std::cout << "Received Command Halt" << std::endl;
        current_state = rov::states::ID::halt;
    }
    option = request->motion_type;

    response->res = "good";
    //RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->res.c_str());
}

ROVController::~ROVController() { }


} // namespace rov
