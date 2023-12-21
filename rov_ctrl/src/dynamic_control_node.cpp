#include "rclcpp/rclcpp.hpp"

#include "rov_msgs/terminal_utils.hpp"
#include "rov_ctrl/dynamic_rov_controller.hpp"
#include "rml/RML.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Name of conf file
    std::string filename = "dcl_rov.conf";
    auto dynamicController = std::make_shared<rov::DynamicRovController>(filename);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(dynamicController);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}
