
#include "rov_ctrl/kinematic_rov_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jsoncpp/json/json.h>

//#include "ulisse_ctrl/configuration.hpp"
//#include "ulisse_ctrl/states/generic_state.hpp"
//#include "ulisse_ctrl/ulisse_defines.hpp"

//#include "ulisse_msgs/terminal_utils.hpp"
#include "rov_msgs/topicnames.hpp"
#include "rov_ctrl/rov_defines.hpp"

//using std::placeholders::_1;
//using std::placeholders::_2;
//using std::placeholders::_3;

namespace rov {

ROVController::ROVController(std::string conf_filename)
    : Node("kinematic_control_node")
    , boundariesSet_(false)
{
    std::cout << "Welcome to ROV controller! " << std::endl;
    std::cout << std::endl;

    //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr client = node->create_client<rov_msgs::srv::UserInput>("user_input");
    cliUserInput_ = this->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);


}

void ROVController::Run(){
    std::cout << "Forward:8, Backward:2, Left:4, Right:6, Up:9, Down:3 " << std::endl;
    std::cout << "Insert a number to change ROV motion: " << std::endl;

    int x;
    std::cin >> x;
    auto request = std::make_shared<rov_msgs::srv::UserInput::Request>();
    //std::make_shared<rov_msgs::srv::UserInput::Request> request;

    request->motion_type = x;

    auto result = cliUserInput_->async_send_request(request);
    //if (rclcpp::spin_until_future_complete(this, result) ==
    //    rclcpp::FutureReturnCode::SUCCESS)
    //{
    //    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "res: ", result.get()->res);
    //    std::cout << "request sent! "<< std::endl;
    //} else {
    //    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    //    std::cout << "failed.."<< std::endl;
    //}

    rclcpp::shutdown();
}


ROVController::~ROVController() { }


} // namespace rov
