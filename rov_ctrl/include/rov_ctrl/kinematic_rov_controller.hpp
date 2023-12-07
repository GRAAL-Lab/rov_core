#ifndef ROV_CTRL_VEHICLECONTROLLER_HPP
#define ROV_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rov_msgs/srv/user_input.hpp"


namespace rov {

class ROVController : public rclcpp::Node {


    /// ROBOT MODEL


    double timestamp_;
    bool boundariesSet_;

    std::chrono::system_clock::time_point tNow_;

    std::string boundariesJson_;

    // service
    rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr cliUserInput_;
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr client = node->create_client<rov_msgs::srv::UserInput>("user_input");

    //ctb::LatLong centroidLocation_;

    //std::shared_ptr<ControlData> ctrlData_;
    //std::shared_ptr<ctb::LatLong> vehiclePosition_;
    //std::shared_ptr<Eigen::Vector2d> inertialF_waterCurrent_;


public:
    ROVController(std::string conf_filename);
    virtual ~ROVController();
    void Run();

};
}
#endif // ROV_CTRL_VEHICLECONTROLLER_HPP
