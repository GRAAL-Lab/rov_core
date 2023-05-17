#include "rclcpp/rclcpp.hpp"
#include "rov_sim/rov_simulator.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    std::string filename = "simulator_rov.conf";

    auto vehicleSimulator = std::make_shared<rov::VehicleSimulator>(filename);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(vehicleSimulator);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
