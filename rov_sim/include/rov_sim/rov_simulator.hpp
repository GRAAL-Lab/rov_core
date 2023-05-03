#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <random>

/*#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/dvl_data.hpp"
#include "ulisse_msgs/msg/fog_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"

#include "ulisse_msgs/msg/llc_thrusters.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"

#include "ulisse_sim/simulator_defines.hpp" */
//home/graal/ros2_ws/src/marine_vehicle_models/surface_vehicle_model/include/surface_vehicle_model.hpp
//home/graal/ros2_ws/src/marine_vehicle_models/underwater_vehicle_model/include/underwater_vehicle_model.hpp

#include "rov_sim/simulator_defines.hpp"
#include "underwater_vehicle_model/underwater_vehicle.hpp"

#include "rov_msgs/msg/simulated_system.hpp"
#include "rov_msgs/msg/micro_loop_count.hpp"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

namespace rov {

class VehicleSimulator : public rclcpp::Node {

    rclcpp::TimerBase::SharedPtr runTimer_;

    GeographicLib::Geodesic geod_;

    double Ts_, Ts_fixed_;
    std::chrono::system_clock::time_point t_start_, t_now_, t_last_;
    std::chrono::nanoseconds iter_elapsed_, total_elapsed_;

    rml::EulerRPY bodyF_orientation_, previous_bodyF_orientation_;
    Eigen::Vector6d bodyF_relativeVelocity_, worldF_relativeVelocity_, worldF_velocity_, worldF_waterVelocity_;
    Eigen::Vector6d bodyF_relativeAcceleration_, worldF_relativeAcceleration_, bodyF_relativeAcceleration_projected_, bodyF_wavesEffects_;

    ctb::LatLong vehiclePos, vehiclePreviousPos, centroidLocation;
    double altitude_;

    Eigen::Matrix3d P_;
    Eigen::Matrix6d bodyF_projection_;

    Eigen::Vector3d bodyF_wFk_;

    double vehicleTrack_, vehicleSpeed_;

    double n_p_, n_s_;
    Eigen::Vector6d volt_cmd; // volt given to rov thrusters

    uint32_t timestamp_count_; // [200Hz counter]
    uint32_t stepssincepps_count_;

    rov_msgs::msg::MicroLoopCount microLoopCountMsg_;
    rov_msgs::msg::SimulatedSystem groundTruthMsg_;
    /* ulisse_msgs::msg::MicroLoopCount microLoopCountMsg_;
    ulisse_msgs::msg::GPSData gpsMsg_;
    ulisse_msgs::msg::Compass compassMsg_;
    ulisse_msgs::msg::IMUData imuMsg_;
    //ulisse_msgs::msg::IMUData orientusMgs_;
    ulisse_msgs::msg::DVLData dvlMsg_;
    ulisse_msgs::msg::FOGData fogMsg_;
    ulisse_msgs::msg::AmbientSensors ambsensMsg_;
    ulisse_msgs::msg::Magnetometer magnetometerMsg_;
    ulisse_msgs::msg::ThrustersReference appliedMotorRefMsg_;

    ulisse_msgs::msg::LLCThrusters motorsDataMsg_;

    rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr microLoopCountPub_;
    rclcpp::Publisher<ulisse_msgs::msg::GPSData>::SharedPtr gpsPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compassPub_;
    rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imuPub_;
    rclcpp::Publisher<ulisse_msgs::msg::DVLData>::SharedPtr dvlPub_;
    rclcpp::Publisher<ulisse_msgs::msg::FOGData>::SharedPtr fogPub_;
    rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsensPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometerPub_;
    rclcpp::Publisher<ulisse_msgs::msg::ThrustersReference>::SharedPtr appliedMotorRefPub_;

    rclcpp::Publisher<ulisse_msgs::msg::LLCThrusters>::SharedPtr motorsDataPub_;
    
    rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrustersSub_;

    int gpsPubCounter_, compassPubCounter_, imuPubCounter_, magnetometerPubCounter_, ambientPubCounter_;
    int orientusPubCounter_, dvlPubCounter_, fogPubCounter_; */

    futils::Timer motorTimeout_;
    rclcpp::Publisher<rov_msgs::msg::SimulatedSystem>::SharedPtr simulatedSystemPub_;

    double hp_, hs_;

    bool realTime_;

    Eigen::RotationMatrix worldF_R_bodyF_;

    std::shared_ptr<SimulatorConfiguration> config_;
    Underwater_Vehicle rovModel_;

    bool LoadConfiguration(const std::string file_name);
    void SimulateActuation();

public:
    VehicleSimulator(const std::string file_name);

    void SetSampleTime(double ts);
    void Run();
    void ExecuteStep();
    void SimulateSensors();
    void PublishSensors();

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

    //void ThrustersReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);
};
}

#endif // VEHICLESIMULATOR_H
