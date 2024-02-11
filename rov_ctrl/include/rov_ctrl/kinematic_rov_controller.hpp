#ifndef ROV_CTRL_VEHICLECONTROLLER_HPP
#define ROV_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rov_msgs/srv/control_command.hpp"
#include "rov_msgs/srv/user_input.hpp"

#include "rov_msgs/msg/reference_velocities.hpp"
#include "rov_msgs/msg/vehicle_status.hpp"
#include "rov_msgs/msg/nav_filter_data.hpp"
#include <string>
#include <fsm/fsm.h>
#include "rov_ctrl/ctrl_data_structs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "rov_ctrl/commands/command_halt.hpp"
#include "rov_ctrl/commands/command_hold.hpp"
#include "rov_ctrl/commands/command_latlong.hpp"
#include "rov_ctrl/commands/command_surgeyawrate.hpp"

#include "rov_ctrl/events/event_near_goal_position.hpp"
#include "rov_ctrl/events/event_rc_enabled.hpp"

#include "rov_ctrl/states/state_halt.hpp"
#include "rov_ctrl/states/state_hold.hpp"
#include "rov_ctrl/states/state_latlong.hpp"
#include "rov_ctrl/states/state_surgeyawrate.hpp"

#include "rov_ctrl/tasks/SafetyBoundaries.hpp"


namespace rov {

class ROVController : public rclcpp::Node {

    TasksInfo taskInfo_;
    std::unordered_map<std::string, TasksInfo> tasksMap_;
    std::unordered_map<std::string, std::shared_ptr<states::GenericState>> statesMap_;
    std::unordered_map<std::string, commands::GenericCommand&> commandsMap_;

    /// ROBOT MODEL
    std::shared_ptr<rml::RobotModel> robotModel_;

    /// Action Manager definition
    std::shared_ptr<tpik::ActionManager> actionManager_;
    std::shared_ptr<tpik::iCAT> iCat_;

    std::shared_ptr<tpik::Solver> solver_;
    // Solution of TPIK
    Eigen::VectorXd yTpik_;

    ///TASKS
    std::shared_ptr<ikcl::LinearVelocity> rovLinearVelocity_;
    std::shared_ptr<ikcl::AngularVelocity> rovAngularVelocity_; //new for ROV
    std::shared_ptr<ikcl::LinearVelocity> rovLinearVelocityHold_;
    std::shared_ptr<ikcl::AlignToTarget> rovAngularPosition_;
    std::shared_ptr<ikcl::CartesianDistance> rovCartesianDistance_;
    std::shared_ptr<ikcl::SafetyBoundaries> rovSafetyBoundaries_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> rovAbsoluteAxisAlignment_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> rovAbsoluteAxisAlignmentSafety_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> rovAbsoluteAxisAlignmentHold_;
    std::shared_ptr<ikcl::CartesianDistance> rovCartesianDistancePathFollowing_;

    double timestamp_;
    bool boundariesSet_;

    std::string fileName_;
    std::chrono::system_clock::time_point tNow_;

    std::string boundariesJson_;

    // service
    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr cliUserInput_;
    rclcpp::Service<rov_msgs::srv::ControlCommand>::SharedPtr srvControlCommand_;
    rclcpp::Service<rov_msgs::srv::UserInput>::SharedPtr srvUserInput_;


    rclcpp::Subscription<rov_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;

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
    int motion_direction;
    Eigen::Vector6d dirV; Eigen::Vector3d bodyF_dirV;
    Eigen::Vector6d reference_speed;

    rov_msgs::msg::ReferenceVelocities referenceVelocities_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped t_stamp_goals;


    // FSM
    fsm::FSM uFsm_;
    std::shared_ptr<states::StateHalt> stateHalt_;
    std::shared_ptr<states::StateHold> stateHold_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateSurgeYawRate> stateSurgeYawRate_;

    commands::CommandHalt commandHalt_;
    commands::CommandHold commandHold_;
    commands::CommandLatLong commandLatLong_;
    commands::CommandSurgeYawRate commandSurgeYawRate_;

    events::EventRCEnabled eventRcEnabled_;
    events::EventNearGoalPosition eventNearGoalPosition_;
/*
    std::shared_ptr<states::StateHalt> stateHalt_;
    std::shared_ptr<states::StateHold> stateHold_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateSurgeHeading> stateSurgeHeading_;
    std::shared_ptr<states::StateSurgeYawRate> stateSurgeYawRate_;
    */
    std::shared_ptr<ControlData> ctrlData_;

    bool LoadConfiguration(std::shared_ptr<KCLConfiguration>& conf);
    void SetUpFSM();
    void SetDirectionVector(Eigen::Vector6d &d_vect);
    void NavFilterCB(const rov_msgs::msg::NavFilterData::SharedPtr msg);
    void PublishLog(std::string log);
public:
    ROVController(std::string conf_filename);
    virtual ~ROVController();
    void Run();
    void PublishControl();
    void PublishTasksInfo();
    void PublishTF();
    void CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<rov_msgs::srv::ControlCommand::Request> request,
                         std::shared_ptr<rov_msgs::srv::ControlCommand::Response> response);
    void userInputHandler(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<rov_msgs::srv::UserInput::Request> request,
                         std::shared_ptr<rov_msgs::srv::UserInput::Response> response);

};
}
#endif // ROV_CTRL_VEHICLECONTROLLER_HPP
