#ifndef NAV_FILTER_ROV_HPP
#define NAV_FILTER_ROV_HPP

#include <cstdlib>
#include <queue>
#include <rml/RML.h>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rov_msgs/msg/compass.hpp"
#include "rov_msgs/msg/gps_data.hpp"
#include "rov_msgs/msg/imu_data.hpp"
#include "rov_msgs/msg/magnetometer.hpp"
#include "nav_filter_rov/nav_data_structs.hpp"
#include "rov_msgs/msg/nav_filter_data.hpp"
/*


#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/llc_thrusters.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"

#include "ulisse_driver/GPSDHelperDataStructs.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ctrl_toolbox/kalman_filter/ExtendedKalmanFilter.h"
#include "ctrl_toolbox/HelperFunctions.h"

#include "surface_vehicle_model/surfacevehiclemodel.hpp"
*/
#include "rov_model/rov_model.hpp"
#include "rov_msgs/msg/simulated_system.hpp"
#include "rov_msgs/topicnames.hpp"
#include <libconfig.h++>

/*
#include "nav_filter_rov/kalman_filter/measurements/accelerometer.hpp"
#include "nav_filter_rov/kalman_filter/measurements/compass.hpp"
#include "nav_filter_rov/kalman_filter/measurements/gps.hpp"
#include "nav_filter_rov/kalman_filter/measurements/gyro.hpp"
#include "nav_filter_rov/kalman_filter/measurements/magnetometer.hpp"
#include "nav_filter_rov/kalman_filter/measurements/z_meter.hpp"
#include "nav_filter_rov/kalman_filter/measurements/rpm.hpp"
#include "nav_filter_rov/kalman_filter/rov_vehicle_model.hpp"
#include "nav_filter_rov/luenberger_observer/pos_vel_observer.hpp"
*/


namespace rov {

namespace nav {

    class NavigationFilter  : public rclcpp::Node {

        std::string confPath_;

        rclcpp::Publisher<rov_msgs::msg::NavFilterData>::SharedPtr navDataPub_;

        rclcpp::Subscription<rov_msgs::msg::Compass>::SharedPtr compassSub_;
        rclcpp::Subscription<rov_msgs::msg::GPSData>::SharedPtr gpsdataSub_;
        rclcpp::Subscription<rov_msgs::msg::IMUData>::SharedPtr imudataSub_;
        rclcpp::Subscription<rov_msgs::msg::Magnetometer>::SharedPtr magnetometerSub_;
        /*
        rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrustersAppliedRefSub_;
        rclcpp::Subscription<ulisse_msgs::msg::LLCThrusters>::SharedPtr llcThrustersSub_;
        rclcpp::Subscription<ulisse_msgs::msg::SimulatedVelocitySensor>::SharedPtr simulatedVelocitySub_;
        */
        
        rclcpp::Subscription<rov_msgs::msg::SimulatedSystem>::SharedPtr simulatedSystemSub_;

        rov_msgs::msg::Compass compassData_;
        rov_msgs::msg::GPSData gpsData_;
        rov_msgs::msg::IMUData imuData_;
        rov_msgs::msg::Magnetometer magnetometerData_;

        rclcpp::TimerBase::SharedPtr runTimer_;
        /*rclcpp::Service<ulisse_msgs::srv::NavFilterCommand>::SharedPtr navFilterCmdService_;        
        rclcpp::TimerBase::SharedPtr sensorsCheckTimer_;


        ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor_;
        ulisse_msgs::msg::ThrustersReference thrustersPercReference_;

        ulisse_msgs::msg::LLCThrusters llcThrustersData_;*/
        rov_msgs::msg::SimulatedSystem simulatedData_;
/*
        double lastValidGPSTime_;
        double lastValidImuTime_;
        double lastValidCompassTime_;
        double lastValidMagnetomerTime_;
        double lastValidLeftRPMTime_;
        double lastValidRightRPMTime_;
        bool gpsValid_, imuValid_, compassValid_, magnetometerValid_, leftRPMValid_, rightRPMValid_;

        int sensorsCheckInterval_;
*/
        NavigationFilterParams filterParams_;
        //Eigen::Vector2d yawRateFilterGains_;

        // Kalman defines
        //std::shared_ptr<UlisseVehicleModel> ulisseModelEKF_; //kalman filter model
        //UlisseVehicleModel::Version ulisseModelVersion_;

        // Measurements
        //std::shared_ptr<GpsMeasurement> gpsMeasurement_;
        //std::shared_ptr<CompassMeasurement> compassMeasurement_;
        //std::shared_ptr<AccelerometerMeasurement> accelerometerMeasurement_;
        //std::shared_ptr<MagnetometerMeasurement> magnetometerMeasurement_;
        //std::shared_ptr<GyroMeasurement> gyroMeasurement_;
        //std::shared_ptr<zMeter> zMeterMeasurement_;
        //std::shared_ptr<RPMMeasurement> portRPMMeasurement_;
        //std::shared_ptr<RPMMeasurement> stbdRPMMeasurement_;

        //std::shared_ptr<ctb::ExtendedKalmanFilter> extendedKalmanFilter_;
        //std::unordered_map<std::string, bool> measuresActive_;
        //int stateDim_;
        //ctb::LatLong centroidLocation_;
        //Eigen::VectorXd state_;
        //std::chrono::system_clock::time_point last_comp_time_;

        //luenberger variables
        //double previousYaw_;
        //double sampleTime_;

        //bool filterEnable_;
        //bool isFirst_;
        //std::queue<double> fifo_h_p_;
        //std::queue<double> fifo_h_s_;

        rov_msgs::msg::NavFilterData filterData_;
        //Eigen::Vector3d NED_gps_cartesian_;

        void LuenbergerObserverFilter();

        void ExtendedKalmanFilter();

        void GroundThruthFilter();

        void SensorsValidityCheck();

        bool LoadConfiguration();

        bool KalmanFilterConfiguration(libconfig::Config& confObj);

        bool LuenbergerObserverConfiguration(libconfig::Config& confObj);

        void GroundTruthDataCB(const rov_msgs::msg::SimulatedSystem::SharedPtr msg);

        void CompassDataCB(const rov_msgs::msg::Compass::SharedPtr msg);

        void GPSDataCB(const rov_msgs::msg::GPSData::SharedPtr msg);

        void IMUDataCB(const rov_msgs::msg::IMUData::SharedPtr msg);

        void MagnetometerDataCB(const rov_msgs::msg::Magnetometer::SharedPtr msg);

        //void CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        //    const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
        //    std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response);
/*
        void ResetFilter();

                            void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

                            void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

                            void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);

                            void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);

                            void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg);

                            void ThrustersAppliedReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);

                            void GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg);

                            void LLCThrustersCB(const ulisse_msgs::msg::LLCThrusters::SharedPtr msg);





*/


    public:
        NavigationFilter(const std::string& confPath);
        virtual ~NavigationFilter();
        void Run();

    };

}
}


#endif /* NAV_FILTER_NAVIGATION_FILTER_HPP */
