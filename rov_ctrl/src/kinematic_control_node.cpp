#include "rclcpp/rclcpp.hpp"
//#include "ulisse_msgs/terminal_utils.hpp"
#include "rov_ctrl/kinematic_rov_controller.hpp"
#include "rml/RML.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string filename = "kcl_rov.conf";
    auto rovController = std::make_shared<rov::ROVController>(filename);

    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(rovController);
    exe.spin();

    //rclcpp::spin(vehicleController);

    rclcpp::shutdown();

    return 0;
}
