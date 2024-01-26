#include "nav_filter_rov/navigation_filter_rov.hpp"
#include <unistd.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rov {

namespace nav {
NavigationFilter::NavigationFilter(const std::string& confPath)
    : Node("navigation_filter_node")
    , confPath_(confPath)
{
    //stateDim_ = 0;
    //centroidLocation_ = ctb::LatLong(44.0956, 9.8631);

    //gyroMeasurement_ = std::make_shared<ulisse::nav::GyroMeasurement>(ulisse::nav::GyroMeasurement());
    //compassMeasurement_ = std::make_shared<ulisse::nav::CompassMeasurement>(ulisse::nav::CompassMeasurement());
    //accelerometerMeasurement_ = std::make_shared<ulisse::nav::AccelerometerMeasurement>(ulisse::nav::AccelerometerMeasurement());
    //gpsMeasurement_ = std::make_shared<ulisse::nav::GpsMeasurement>(ulisse::nav::GpsMeasurement());

    /*magnetometerMeasurement_ = std::make_shared<ulisse::nav::MagnetometerMeasurement>(ulisse::nav::MagnetometerMeasurement());
    zMeterMeasurement_ = std::make_shared<ulisse::nav::zMeter>(ulisse::nav::zMeter());
    portRPMMeasurement_ = std::make_shared<ulisse::nav::RPMMeasurement>(ulisse::nav::RPMMeasurement());
    stbdRPMMeasurement_ = std::make_shared<ulisse::nav::RPMMeasurement>(ulisse::nav::RPMMeasurement());

    portRPMMeasurement_->SetPortStarboard(ulisse::nav::Side::Port);
    stbdRPMMeasurement_->SetPortStarboard(ulisse::nav::Side::Starboard);
    */

    //Load filter params
    if (!LoadConfiguration()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load navigation filter configuration");
        exit(EXIT_FAILURE);
    }

    //Publisher of nav data structure
    navDataPub_ = this->create_publisher<rov_msgs::msg::NavFilterData>(rov_msgs::topicnames::nav_filter_data, 1);


    //Subscribes to data sensors
    simulatedSystemSub_ = this->create_subscription<rov_msgs::msg::SimulatedSystem>(rov_msgs::topicnames::simulated_system,1, std::bind(&NavigationFilter::GroundTruthDataCB, this, _1));

    compassSub_ = this->create_subscription<rov_msgs::msg::Compass>(rov_msgs::topicnames::sensor_compass,
        1, std::bind(&NavigationFilter::CompassDataCB, this, _1));
    gpsdataSub_ = this->create_subscription<rov_msgs::msg::GPSData>(rov_msgs::topicnames::sensor_gps_data,
        1, std::bind(&NavigationFilter::GPSDataCB, this, _1));
    imudataSub_ = this->create_subscription<rov_msgs::msg::IMUData>(rov_msgs::topicnames::sensor_imu,
        1, std::bind(&NavigationFilter::IMUDataCB, this, _1));
    magnetometerSub_ = this->create_subscription<rov_msgs::msg::Magnetometer>(rov_msgs::topicnames::sensor_magnetometer,
        1, std::bind(&NavigationFilter::MagnetometerDataCB, this, _1));
   /*
    thrustersAppliedRefSub_ = this->create_subscription<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_applied_perc,
        1, std::bind(&NavigationFilter::ThrustersAppliedReferenceCB, this, _1));
    llcThrustersSub_ = this->create_subscription<ulisse_msgs::msg::LLCThrusters>(ulisse_msgs::topicnames::llc_thrusters,
        1, std::bind(&NavigationFilter::LLCThrustersCB, this, _1));

    simulatedVelocitySub_ = this->create_subscription<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor,
        1, std::bind(&NavigationFilter::SimulatedVelocitySensorCB, this, _1));

    lastValidGPSTime_ = 0.0;
    lastValidImuTime_ = 0.0;
    lastValidCompassTime_ = 0.0;
    lastValidMagnetomerTime_ = 0.0;
*/
    filterData_.gps_received = filterData_.imu_received = filterData_.compass_received = filterData_.magnetometer_received = false;

    //previousYaw_ = 0.0;
    //sampleTime_ = 0.0;

    int msRunPeriod = 1.0 / (filterParams_.rate) * 1000;
    RCLCPP_INFO(this->get_logger(), "NavFilter Rate: %d Hz", filterParams_.rate);
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&NavigationFilter::Run, this));


}

NavigationFilter::~NavigationFilter() { }

void NavigationFilter::Run()
{

    SensorsValidityCheck();

    if (filterParams_.mode == FilterMode::LuenbergerObserver) {
        LuenbergerObserverFilter();
    } else if (filterParams_.mode == FilterMode::KalmanFilter) {
        ExtendedKalmanFilter();
    } else if (filterParams_.mode == FilterMode::GroundTruth) {
        GroundThruthFilter();
    }

    navDataPub_->publish(filterData_);

    //auto tNow = std::chrono::system_clock::now();
    //long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
    //auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    //auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
    //std_msgs::msg::Float64 msg;
    //msg.stamp.sec = now_stamp_secs;
    //msg.stamp.nanosec = now_stamp_nanosecs;

    //Eigen::Vector3d bodyframe_velocity = { filterData_.bodyframe_linear_velocity[0], filterData_.bodyframe_linear_velocity[1], filterData_.bodyframe_linear_velocity[2] };
    //Eigen::Vector3d inertialframe_current = { filterData_.inertialframe_water_current[0], filterData_.inertialframe_water_current[1], 0 };
    //rml::EulerRPY rpy { filterData_.bodyframe_angular_position.roll, filterData_.bodyframe_angular_position.pitch, filterData_.bodyframe_angular_position.yaw };



}

void NavigationFilter::SensorsValidityCheck(){

}

void NavigationFilter::LuenbergerObserverFilter()
{

}

void NavigationFilter::ExtendedKalmanFilter()
{

}

void NavigationFilter::GroundThruthFilter()
{
    auto tNow = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    filterData_.stamp.sec = now_stamp_secs;
    filterData_.stamp.nanosec = now_stamp_nanosecs;

    filterData_.inertialframe_linear_position.latlong.latitude = simulatedData_.inertialframe_linear_position.latlong.latitude;
    filterData_.inertialframe_linear_position.latlong.longitude = simulatedData_.inertialframe_linear_position.latlong.longitude;
    filterData_.inertialframe_linear_position.altitude = simulatedData_.inertialframe_linear_position.altitude;
    filterData_.bodyframe_angular_position.roll = simulatedData_.bodyframe_angular_position.roll;
    filterData_.bodyframe_angular_position.pitch = simulatedData_.bodyframe_angular_position.pitch;
    filterData_.bodyframe_angular_position.yaw = simulatedData_.bodyframe_angular_position.yaw;
    filterData_.bodyframe_linear_velocity[0] = simulatedData_.bodyframe_linear_velocity[0];
    filterData_.bodyframe_linear_velocity[1] = simulatedData_.bodyframe_linear_velocity[1];
    filterData_.bodyframe_linear_velocity[2] = simulatedData_.bodyframe_linear_velocity[2];

    filterData_.bodyframe_angular_velocity[0] = simulatedData_.bodyframe_angular_velocity[0];
    filterData_.bodyframe_angular_velocity[1] = simulatedData_.bodyframe_angular_velocity[1];
    filterData_.bodyframe_angular_velocity[2] = simulatedData_.bodyframe_angular_velocity[2];
    filterData_.inertialframe_water_current[0] = simulatedData_.inertialframe_water_current[0];
    filterData_.inertialframe_water_current[1] = simulatedData_.inertialframe_water_current[1];
    filterData_.inertialframe_water_current[2] = simulatedData_.inertialframe_water_current[2];
    filterData_.gyro_bias[0] = simulatedData_.gyro_bias[0];
    filterData_.gyro_bias[1] = simulatedData_.gyro_bias[1];
    filterData_.gyro_bias[2] = simulatedData_.gyro_bias[2];
}


bool NavigationFilter::LoadConfiguration()
{
    libconfig::Config confObj;

    // Read the blueROV2 MODEL config file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("underwater_vehicle_model");
    std::string modelConfPath = package_share_directory + "/conf/blueROV.conf";

    RCLCPP_INFO(this->get_logger(), "PATH TO blueROV2_MODEL CONF FILE (NAV): %s", modelConfPath.c_str());
    try {
        confObj.readFile(modelConfPath.c_str());
    } catch (libconfig::ParseException& e) {
        RCLCPP_ERROR(this->get_logger(), "Parse exception when reading: %s", modelConfPath.c_str());
        RCLCPP_ERROR(this->get_logger(), "Line: %d - Error: %s", e.getLine(), e.getError());
        return false;
    }

    //SurfaceVehicleModelParameters ulisseModelParams;
    UnderwaterModelParameters rovModelParams;

    if (!rovModelParams.LoadConfiguration(confObj)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load BlueROV2 model");
        return false;
    }

    // Read the NAV_FILTER config file
    try {
        confObj.readFile(confPath_.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        RCLCPP_ERROR(this->get_logger(), "I/O error while reading file: %s", fioex.what());
        return false;
    } catch (libconfig::ParseException& e) {
        RCLCPP_ERROR(this->get_logger(), "Parse exception when reading: %s", confPath_.c_str());
        RCLCPP_ERROR(this->get_logger(), "Line: %d - Error: %s", e.getLine(), e.getError());
        return false;
    } catch (libconfig::SettingException& e) {
        std::cerr << "ACSADASDA" << std::endl;
        RCLCPP_ERROR(this->get_logger(), "Setting error while reading %s: %s", e.getPath(), e.what());
    }

    // Configure the filter node params
    if (!filterParams_.ConfigureFromFile(confObj)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load navigation mode/rate params");
        return false;
    };

    if (filterParams_.mode == FilterMode::LuenbergerObserver) {
        if (!LuenbergerObserverConfiguration(confObj)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load Luenberger Observer configuration");
            return false;
        };

    } else if (filterParams_.mode == FilterMode::KalmanFilter) {
        if (!KalmanFilterConfiguration(confObj)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load Kalman Filter configuration");
            return false;
        }
        //ulisseModelEKF_->ModelParameters() = ulisseModelParams;
    } else if (filterParams_.mode == FilterMode::GroundTruth) {

    } else {
        RCLCPP_ERROR(this->get_logger(), "Type of filter not recognized");
        return false;
    }

    return true;
}

bool NavigationFilter::KalmanFilterConfiguration(libconfig::Config& confObj) noexcept(false){
    return true;
}

bool NavigationFilter::LuenbergerObserverConfiguration(libconfig::Config& confObj) noexcept(false){
    return true;
}

void NavigationFilter::GroundTruthDataCB(const rov_msgs::msg::SimulatedSystem::SharedPtr msg) { simulatedData_ = *msg; }

void NavigationFilter::CompassDataCB(const rov_msgs::msg::Compass::SharedPtr msg) { compassData_ = *msg; }

void NavigationFilter::GPSDataCB(const rov_msgs::msg::GPSData::SharedPtr msg) { gpsData_ = *msg; }

void NavigationFilter::IMUDataCB(const rov_msgs::msg::IMUData::SharedPtr msg) { imuData_ = *msg; /*RCLCPP_INFO(this->get_logger(), "IMU Callback()");*/ }

void NavigationFilter::MagnetometerDataCB(const rov_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData_ = *msg; }


}
}
