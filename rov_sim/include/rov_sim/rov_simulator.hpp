#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

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
#include "rov_msgs/msg/winch_motor_reference.hpp"

#include "rov_sim/simulator_defines.hpp"
#include "rov_model/rov_model.hpp"

#include "rov_msgs/msg/simulated_system.hpp"
#include "ulisse_msgs/msg/simulated_system.hpp"

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

class VehicleSimulator : public rclcpp::Node {

    rclcpp::TimerBase::SharedPtr runTimer_;

    GeographicLib::Geodesic geod_;

    double Ts_, Ts_fixed_;
    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    rml::EulerRPY bodyF_orientation_, previous_bodyF_orientation_;
    rml::EulerRPY bodyF_ROVmesh_;
    Eigen::Vector6d bodyF_relativeVelocity_, worldF_relativeVelocity_, worldF_velocity_, worldF_waterVelocity_;
    Eigen::Vector6d bodyF_relativeAcceleration_, worldF_relativeAcceleration_, bodyF_relativeAcceleration_projected_, bodyF_wavesEffects_;

    ctb::LatLong vehiclePos_, vehiclePreviousPos_, centroidLocation_;
    double altitude_, Pre_altitude_;
    Eigen::Vector3d ROVpose_,ROVprepose_;
    Eigen::Vector3d centerUTM_;

    // cable variable
    ctb::LatLong cableStartPos_, cableEndPos_;
    float cableLength_, ref_cableLength_;
    double cableStart_altitude_, cableEnd_altitude_;
    Eigen::Vector3d bodyF_cable_ending_, bodyF_cable_starting_;

    Eigen::Vector3d cableStart_cartesian_;
    Eigen::Vector3d cableEnd_cartesian_;


    //Eigen::Matrix3d P_;
    Eigen::Matrix6d bodyF_projection_;

    Eigen::Vector3d bodyF_wFk_;

    double vehicleTrack_, vehicleSpeed_;

    //double n_p_, n_s_;
    Eigen::VectorXd volt_cmd; // volt given to rov thrusters

    uint32_t timestamp_count_; // [200Hz counter]
    uint32_t stepssincepps_count_;

    rov_msgs::msg::MicroLoopCount microLoopCountMsg_;
    rov_msgs::msg::SimulatedSystem groundTruthMsg_;
    rov_msgs::msg::SimulatedSystem groundTruth_UlisseMsg_;
    rov_msgs::msg::Forces forcesMsg_;
    rov_msgs::msg::CableData cableMsg_;
    rov_msgs::msg::WinchMotorReference winchMotorReferenceMsg_;
    //rov_msgs::msg::CableLengthReference cableRefMsg_;

    geometry_msgs::msg::PoseStamped pt_;

    geometry_msgs::msg::TransformStamped t_stamp;
    geometry_msgs::msg::TransformStamped t_stamp_ROV;

    // service
    //rclcpp::Service<rov_msgs::srv::UserInput>::SharedPtr srvUserInput_;

    rov_msgs::msg::GPSData gpsMsg_;
    rov_msgs::msg::Compass compassMsg_;
    rov_msgs::msg::IMUData imuMsg_;
    rov_msgs::msg::Magnetometer magnetometerMsg_;
    rov_msgs::msg::PressureData pressureMsg_;

    rclcpp::Publisher<rov_msgs::msg::GPSData>::SharedPtr gpsPub_;
    rclcpp::Publisher<rov_msgs::msg::Compass>::SharedPtr compassPub_;
    rclcpp::Publisher<rov_msgs::msg::IMUData>::SharedPtr imuPub_;
    rclcpp::Publisher<rov_msgs::msg::Magnetometer>::SharedPtr magnetometerPub_;
    rclcpp::Publisher<rov_msgs::msg::PressureData>::SharedPtr pressurePub_;

    /*
    rclcpp::Publisher<ulisse_msgs::msg::DVLData>::SharedPtr dvlPub_;
    rclcpp::Publisher<ulisse_msgs::msg::FOGData>::SharedPtr fogPub_;
    rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsensPub_;

    rclcpp::Publisher<ulisse_msgs::msg::ThrustersReference>::SharedPtr appliedMotorRefPub_;

    rclcpp::Publisher<ulisse_msgs::msg::LLCThrusters>::SharedPtr motorsDataPub_;

    rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrustersSub_;

    int gpsPubCounter_, compassPubCounter_, imuPubCounter_, magnetometerPubCounter_, ambientPubCounter_;
    int orientusPubCounter_, dvlPubCounter_, fogPubCounter_; */
    int gpsPubCounter_, compassPubCounter_, imuPubCounter_, magnetometerPubCounter_, pressurePubConter_;// ambientPubCounter_;

    futils::Timer motorTimeout_;
    rclcpp::Publisher<rov_msgs::msg::SimulatedSystem>::SharedPtr simulatedSystemPub_;
    rclcpp::Publisher<rov_msgs::msg::MicroLoopCount>::SharedPtr microLoopCountPub_;
    rclcpp::Publisher<rov_msgs::msg::Forces>::SharedPtr forcesPub_;
    rclcpp::Publisher<rov_msgs::msg::CableData>::SharedPtr cableDataPub_;

    rclcpp::Subscription<rov_msgs::msg::ThrustersReference>::SharedPtr thrustersSub_;
    rclcpp::Subscription<rov_msgs::msg::CableLengthReference>::SharedPtr winchSub_;
    rclcpp::Subscription<rov_msgs::msg::WinchMotorReference>::SharedPtr winchMotorRefSub_;
    rclcpp::Subscription<ulisse_msgs::msg::SimulatedSystem>::SharedPtr simulatedSystemAsvSub_;

    //rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ROV;

    double hp_, hs_;

    bool realTime_;

    Eigen::RotationMatrix worldF_ROV_bodyF_;
    Eigen::RotationMatrix worldF_ASV_bodyF_;
    Eigen::RotationMatrix worldF_ROV_meshF_;

    std::shared_ptr<SimulatorConfiguration> config_;
    Rov rovModel_;

    // Rviz
    visualization_msgs::msg::Marker vehicleMarker_;

    int option; // motion of ROV
    bool ASVmsg;

    bool LoadConfiguration(const std::string file_name);
    void SimulateActuation();

public:
    VehicleSimulator(const std::string file_name);

    void SetSampleTime(double ts);
    void Run();
    void ExecuteStep();
    void SimulateSensors();
    void PublishSensors();
    void PublishTf();

    void AssignMessage(std::array<double,6>& msg, const Eigen::Vector6d& vector);

    auto WorldF_Velocity() const -> const Eigen::Vector6d& { return worldF_velocity_; }
    /*auto Altitude() const -> const rml::EulerRPY& { return bodyF_orientation_; }
    auto Latitude() const -> double { return latitude_; }
    auto Longitude() const -> double { return longitude_; }*/



    /**
     * @brief Set if simulation should run in Realtime or not
     *
     * By default `realtime` is set to true, so the simulator runs in real-time. This means that,
     * even if a Ts (sample time) has been set, the simulator will use actual time differences
     * calculated with std::chrono to simulate time. If instead we set `realtime` to false, we can
     * run the simulator at any frequency, and time will proceed by steps of Ts.
     *
     * @param[in] realtime
     */
    void SetRealtime(bool realtime);
    double GetCurrentTimeStamp() const;

    void ThrustersReferenceCB(const rov_msgs::msg::ThrustersReference::SharedPtr msg);
    void CableLengthReferenceCB(const rov_msgs::msg::CableLengthReference::SharedPtr msg);
    void ASVsimulatedSysCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg);
    void WinchMotorReferenceCB(const rov_msgs::msg::WinchMotorReference::SharedPtr msg);
};
}

#endif // VEHICLESIMULATOR_H
