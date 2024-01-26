#ifndef ROV_CTRL_VEHICLECONTROLLER_HPP
#define ROV_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rov_msgs/srv/control_command.hpp"
#include "rov_msgs/msg/reference_velocities.hpp"
#include "rov_msgs/msg/vehicle_status.hpp"
#include <string>
#include <fsm/fsm.h>
#include "rov_ctrl/ctrl_data_structs.hpp"

namespace rov {

class ROVController : public rclcpp::Node {


    /// ROBOT MODEL


    double timestamp_;
    bool boundariesSet_;

    std::string fileName_;
    std::chrono::system_clock::time_point tNow_;

    std::string boundariesJson_;

    // service
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr cliUserInput_;
    rclcpp::Service<rov_msgs::srv::ControlCommand>::SharedPtr srvControlCommand_;

    rclcpp::Publisher<rov_msgs::msg::ReferenceVelocities>::SharedPtr  referenceVelocitiesPub_;
    rclcpp::Publisher<rov_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr genericLogPub_;

    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr client = node->create_client<rov_msgs::srv::UserInput>("user_input");

    //ctb::LatLong centroidLocation_;

    //std::shared_ptr<ControlData> ctrlData_;
    //std::shared_ptr<ctb::LatLong> vehiclePosition_;
    //std::shared_ptr<Eigen::Vector2d> inertialF_waterCurrent_;

    rclcpp::TimerBase::SharedPtr runTimer_;
    std::shared_ptr<KCLConfiguration> conf_;
    ctb::LatLong centroidLocation_;

    int option; // motion of ROV
    std::string current_state;

    // FSM
    /*
    fsm::FSM uFsm_;

    std::shared_ptr<states::StateHalt> stateHalt_;
    std::shared_ptr<states::StateHold> stateHold_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateSurgeHeading> stateSurgeHeading_;
    std::shared_ptr<states::StateSurgeYawRate> stateSurgeYawRate_;
    */

    bool LoadConfiguration(std::shared_ptr<KCLConfiguration>& conf);
    void PublishLog(std::string log);
public:
    ROVController(std::string conf_filename);
    virtual ~ROVController();
    void Run();
    void PublishControl();
    void CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<rov_msgs::srv::ControlCommand::Request> request,
                         std::shared_ptr<rov_msgs::srv::ControlCommand::Response> response);

};
}
#endif // ROV_CTRL_VEHICLECONTROLLER_HPP
