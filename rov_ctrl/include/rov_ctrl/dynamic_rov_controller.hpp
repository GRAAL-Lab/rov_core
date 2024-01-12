#ifndef ROV_CTRL_DYNAMIC_VEHICLE_CONTROLLER_HPP
#define ROV_CTRL_DYNAMIC_VEHICLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ctrl_toolbox/pid/DigitalPID.h"
 /*
#include "ulisse_msgs/msg/dynamic_pid_control.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"

#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thruster_mapping_control.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
//#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"
*/


#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rov_msgs/topicnames.hpp"
#include "rov_msgs/msg/reference_velocities.hpp"
#include "rov_msgs/msg/thrusters_reference.hpp"
#include "rov_msgs/msg/dynamic_pid_control.hpp"
#include "rov_msgs/msg/vehicle_status.hpp"
 #include "rov_msgs/msg/forces.hpp"
#include "underwater_vehicle_model/underwater_vehicle.hpp"
#include "rov_ctrl/ctrl_data_structs.hpp"

namespace rov {

class DynamicRovController : public rclcpp::Node {

    double sampleTime_;
    std::string confFileName_;
    //ulisse_msgs::msg::NavFilterData filterData;
    rov_msgs::msg::ReferenceVelocities referenceVelocities;
    rov_msgs::msg::VehicleStatus vehicleStatus;
    rov_msgs::msg::Forces vehicleForces;

    rclcpp::TimerBase::SharedPtr runTimer_;

    //config struct

    std::shared_ptr<DCLConfiguration> dcl_conf;// = std::make_shared<DCLConfiguration>();

    // ulisse model
    //SurfaceVehicleModel ulisseModel;
    Underwater_Vehicle rovModel_;
    Eigen::MatrixXd rov_allocationMatrix;

    //rclcpp::Service<ulisse_msgs::srv::ResetConfiguration>::SharedPtr srvResetConf_;

    //Subscribers
    //rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr filterSub_;
    //rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;
    rclcpp::Subscription<rov_msgs::msg::ReferenceVelocities>::SharedPtr referenceVelocitiesSub_;
    rclcpp::Subscription<rov_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;
    rclcpp::Subscription<rov_msgs::msg::Forces>::SharedPtr vehicleForcesSub_;

    //Publishers
    rclcpp::Publisher<rov_msgs::msg::ThrustersReference>::SharedPtr thrusterDataPub_;// = this->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc, 1);
    //rclcpp::Publisher<ulisse_msgs::msg::ThrusterMappingControl>::SharedPtr thrusterMappigPub_;// = this->create_publisher<ulisse_msgs::msg::ThrusterMappingControl>(ulisse_msgs::topicnames::thruster_mapping_control, 1);
    //rclcpp::Publisher<ulisse_msgs::msg::SimulatedVelocitySensor>::SharedPtr simulatedVelocitySensorPub_;// = this->create_publisher<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 1);
    rclcpp::Publisher<rov_msgs::msg::DynamicPidControl>::SharedPtr classicPidControlPub_;// = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::classic_pid_control, 1);
    //rclcpp::Publisher<ulisse_msgs::msg::DynamicPidControl>::SharedPtr computedTorqueControlPub_;// = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::computed_torque_control, 1);


    //local variables
    //ulisse_msgs::msg::ThrusterMappingControl thrusterMappingMsg;
    rov_msgs::msg::ThrustersReference thrustersReference;
    rov_msgs::msg::DynamicPidControl classicPidControlMsg;//, computedTorqueMsg;
    //ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor;

    //feedback from nav filter
    //double surgeFbk = 0.0;
    //double yawRateFbk = 0.0;

    //double motorLeft = 0.0, motorRight = 0.0;

    //Surge pid for thrusterMapping control (TM)
    //ctb::DigitalPID pidSurgeTM;

    //Pid for classic pid control (CP)
    ctb::DigitalPID pidYawRateCP;
    ctb::DigitalPID pidSurgeCP;

    //Pid for computed torque control (CT)
    //ctb::DigitalPID pidYawRateCT;
    //ctb::DigitalPID pidSurgeCT;

    //Eigen::Vector6d tau = Eigen::Vector3d::Zero();

    //void ResetConfHandler(const std::shared_ptr<rmw_request_id_t> request_header,
    //    const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
    //    std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response);

    bool LoadDclConfiguration(std::shared_ptr<DCLConfiguration> conf, std::string filename);

    //void ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid);
    void ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate);

    void MoveByForce(const Eigen::Vector6d &force, Eigen::VectorXd &volt);

    //void ComputedTorqueControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate);

    //void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void ReferenceVelocitiesCB(const rov_msgs::msg::ReferenceVelocities::SharedPtr msg);
    void VehicleStatusCB(const rov_msgs::msg::VehicleStatus::SharedPtr msg);
    void VehicleForcesCB(const rov_msgs::msg::Forces::SharedPtr msg);
    //void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);


public:
    DynamicRovController(std::string file_name);
    virtual ~DynamicRovController();

    void Run();
    void PublishControl();

};
}
#endif // ROV_CTRL_DYNAMIC_VEHICLE_CONTROLLER_HPP
