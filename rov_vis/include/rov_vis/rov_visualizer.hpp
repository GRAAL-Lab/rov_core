#ifndef VEHICLEVISUALIZER_H
#define VEHICLEVISUALIZER_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <random>

#include "rov_msgs/msg/compass.hpp"
#include "rov_msgs/msg/gps_data.hpp"
#include "rov_msgs/msg/imu_data.hpp"
#include "rov_msgs/msg/magnetometer.hpp"
#include "rov_msgs/msg/pressure_data.hpp"
#include "rov_msgs/msg/cable_data.hpp"
#include "rov_msgs/msg/cable_length_reference.hpp"
#include "rov_msgs/msg/nav_filter_data.hpp"

//#include "rov_sim/simulator_defines.hpp"
#include "rov_vis/visualizer_defines.hpp"
#include "underwater_vehicle_model/underwater_vehicle.hpp"

#include "rov_msgs/msg/simulated_system.hpp"
//#include "ulisse_msgs/msg/simulated_system.hpp"

#include "rov_msgs/msg/micro_loop_count.hpp"
#include "rov_msgs/msg/forces.hpp"
#include "rov_msgs/msg/thrusters_reference.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

//#include "GeographicLib/Geodesic.hpp"
//#include "eigen3/Eigen/Dense"
//#include "rml/RML.h"

#include "rov_msgs/srv/user_input.hpp"

#include "rov_msgs/topicnames.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker.h"
#include "visualization_msgs/msg/marker_array.hpp"

//#include "visualization_msgs/InteractiveMarker.h"

namespace rov {

class VehicleVisualizer : public rclcpp::Node {

    rclcpp::TimerBase::SharedPtr runTimer_;
    double Ts_, Ts_fixed_;
    bool realTime_;
    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    ctb::LatLong vehiclePos_, vehiclePreviousPos_, centroidLocation_;
    Eigen::Vector3d centerUTM_;

    std::shared_ptr<VisualizerConfiguration> config_;

    geometry_msgs::msg::TransformStamped t_stamp;
    geometry_msgs::msg::TransformStamped t_stamp_ROV;

    rclcpp::Subscription<rov_msgs::msg::NavFilterData>::SharedPtr navDataSub_;
    rov_msgs::msg::NavFilterData navData_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizationPub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ROV;

    bool LoadConfiguration(const std::string file_name);

    void NavDataCB(const rov_msgs::msg::NavFilterData::SharedPtr msg);


public:
    VehicleVisualizer(const std::string file_name);

    void SetSampleTime(double ts);
    void Run();
    void ExecuteStep();
    void AssignMessage(std::array<double,6>& msg, const Eigen::Vector6d& vector);
    void PublishTf();

    double GetCurrentTimeStamp() const;

    
};
}

#endif // VehicleVisualizer_H
