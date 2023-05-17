#include <rclcpp/rclcpp.hpp>
#include <nav_filter/navigation_filter.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string confPath = ament_index_cpp::get_package_share_directory("nav_filter").append("/conf/navigation_filter.conf");
    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    rclcpp::executors::SingleThreadedExecutor exe;
    auto navFilterNode = std::make_shared<ulisse::nav::NavigationFilter>(confPath);
    exe.add_node(navFilterNode);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
