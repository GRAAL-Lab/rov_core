#include <rclcpp/rclcpp.hpp>
//#include <nav_filter_rov/navigation_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_filter_rov/navigation_filter_rov.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string confPath = ament_index_cpp::get_package_share_directory("nav_filter_rov").append("/conf/navigation_filter_rov.conf");
    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    rclcpp::executors::SingleThreadedExecutor exe;
    auto navFilterNode = std::make_shared<rov::nav::NavigationFilter>(confPath);
    exe.add_node(navFilterNode);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
