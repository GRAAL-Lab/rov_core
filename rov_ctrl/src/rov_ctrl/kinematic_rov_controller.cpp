
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

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr client = node->create_client<rov_msgs::srv::UserInput>("user_input");
    cliUserInput_ = this->create_client<rov_msgs::srv::UserInput>("user_input");


}

void ROVController::Run(){
    std::cout << "Forward:8, Backward:2, Left:4, Right:6, Up:9, Down:3 " << std::endl;
    std::cout << "Insert a number: " << std::endl;

    int x;
    std::cin >> x;
    auto request = std::make_shared<rov_msgs::srv::UserInput::Request>();
    request->motion_type = x;

    rclcpp::shutdown();
}


ROVController::~ROVController() { }


} // namespace rov
