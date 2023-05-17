#include <rclcpp/rclcpp.hpp>

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"

#include "ulisse_msgs/futils.hpp"

struct NavFilterData {
    double inertialF_linearPosition[2];
    double bodyF_angularPosition[3];
    double bodyF_linearVelocity[3];
    double bodyF_angularVelocity[3];
    double inertialF_waterCurrent[2];
};

struct GPSData {
    double time;
    double latitude, longitude, altitude;
    double speed;
    double track;
};

struct IMUData {
    double gyroscope[3];
    double accelerometer[3];
};

NavFilterData navFilterData;
GPSData gpsData;
IMUData imuData;
double compassRPY[3];
double magnetometer[3];

ulisse_msgs::msg::NavFilterData navFilterMsg;
ulisse_msgs::msg::GPSData gpsDataMsg;
ulisse_msgs::msg::IMUData imuDataMsg;
ulisse_msgs::msg::Compass compassRPYMsg;
ulisse_msgs::msg::Magnetometer magnetometerMsg;

bool navFilterReceived = false;
bool gpsReceived = false;
bool imuReceived = false;
bool compassReceived = false;
bool magnetometerReceived = false;

void NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
    navFilterMsg = *msg;
    navFilterReceived = true;
}

void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) {
    gpsDataMsg = *msg;
    gpsReceived = true;
}
void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg) {
    imuDataMsg = *msg;
    imuReceived = true;
}
void CompassCB(const ulisse_msgs::msg::Compass::SharedPtr msg) {
    compassRPYMsg = *msg;
    compassReceived = true;
}
void MagnetometerCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) {
    magnetometerMsg = *msg;
    magnetometerReceived = true;
}

int main(int argc, char* argv[])
{
    if(argc < 2){
        std::cerr << "argv[1] missing: No destination IP provided" << std::endl;
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    std::string ip(argv[1]);
    std::string port_nav_filter("8880"), port_gps("8881"), port_imu("8882"), port_compass("8883"), port_magnetometer("8884");
    std::cout << "UDP Configuration IP:ports = " << ip << ":" << port_nav_filter << "-" << port_magnetometer << std::endl;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("nav_filter_udp_sender");
    rclcpp::WallRate loop_rate(50);

    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;
    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gpsDataSub_;
    rclcpp::Subscription<ulisse_msgs::msg::IMUData>::SharedPtr imuDataSub_;
    rclcpp::Subscription<ulisse_msgs::msg::Compass>::SharedPtr compassSub_;
    rclcpp::Subscription<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometerSub_;

    navFilterSub_ = node->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, NavFilterCB);
    gpsDataSub_ = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, GPSDataCB);
    imuDataSub_ = node->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 10, IMUDataCB);
    compassSub_ = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 10, CompassCB);
    magnetometerSub_ = node->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 10, MagnetometerCB);

    futils::UDPSenderSocket udpSenderNavFilter(ip.c_str(), port_nav_filter.c_str());
    futils::UDPSenderSocket udpSenderGPS(ip.c_str(), port_gps.c_str());
    futils::UDPSenderSocket udpSenderIMU(ip.c_str(), port_imu.c_str());
    futils::UDPSenderSocket udpSenderCompass(ip.c_str(), port_compass.c_str());
    futils::UDPSenderSocket udpSenderMagnetometer(ip.c_str(), port_magnetometer.c_str());

    while (rclcpp::ok()) {

        rclcpp::spin_some(node);

        if(navFilterReceived){
            navFilterReceived = false;

            navFilterData.inertialF_linearPosition[0] = navFilterMsg.inertialframe_linear_position.latlong.latitude;
            navFilterData.inertialF_linearPosition[1] = navFilterMsg.inertialframe_linear_position.latlong.longitude;

            navFilterData.bodyF_angularPosition[0] = navFilterMsg.bodyframe_angular_position.roll;
            navFilterData.bodyF_angularPosition[1] = navFilterMsg.bodyframe_angular_position.pitch;
            navFilterData.bodyF_angularPosition[2] = navFilterMsg.bodyframe_angular_position.yaw;

            navFilterData.bodyF_linearVelocity[0] = navFilterMsg.bodyframe_linear_velocity[0];
            navFilterData.bodyF_linearVelocity[1] = navFilterMsg.bodyframe_linear_velocity[1];
            navFilterData.bodyF_linearVelocity[2] = navFilterMsg.bodyframe_linear_velocity[2];

            navFilterData.bodyF_angularVelocity[0] = navFilterMsg.bodyframe_angular_velocity[0];
            navFilterData.bodyF_angularVelocity[1] = navFilterMsg.bodyframe_angular_velocity[1];
            navFilterData.bodyF_angularVelocity[2] = navFilterMsg.bodyframe_angular_velocity[2];

            navFilterData.inertialF_waterCurrent[0] = navFilterMsg.inertialframe_water_current[0];
            navFilterData.inertialF_waterCurrent[1] = navFilterMsg.inertialframe_water_current[1];

            udpSenderNavFilter.Send(&navFilterData, sizeof(navFilterData));
            //std::cout << "Sent UDP navFilterData message" << std::endl;
        }

        if(gpsReceived){
            gpsReceived = false;

            gpsData.time = gpsDataMsg.time;
            gpsData.latitude = gpsDataMsg.latitude;
            gpsData.longitude = gpsDataMsg.longitude;
            gpsData.altitude = gpsDataMsg.altitude;
            gpsData.speed = gpsDataMsg.speed;
            gpsData.track = gpsDataMsg.track;

            udpSenderGPS.Send(&gpsData, sizeof(gpsData));
        }

        if(imuReceived){
            imuReceived = false;
            imuData.accelerometer[0] = imuDataMsg.accelerometer[0];
            imuData.accelerometer[1] = imuDataMsg.accelerometer[1];
            imuData.accelerometer[2] = imuDataMsg.accelerometer[2];

            imuData.gyroscope[0] = imuDataMsg.gyro[0];
            imuData.gyroscope[1] = imuDataMsg.gyro[1];
            imuData.gyroscope[2] = imuDataMsg.gyro[2];

            udpSenderIMU.Send(&imuData,sizeof(imuData));
        }

        if(compassReceived){
            compassReceived = false;
            compassRPY[0] = compassRPYMsg.orientation.roll;
            compassRPY[1] = compassRPYMsg.orientation.pitch;
            compassRPY[2] = compassRPYMsg.orientation.yaw;

            udpSenderCompass.Send(&compassRPY,sizeof(compassRPY));
        }

        if(magnetometerReceived){
            magnetometerReceived = false;
            magnetometer[0] = magnetometerMsg.orthogonalstrength[0];
            magnetometer[1] = magnetometerMsg.orthogonalstrength[1];
            magnetometer[2] = magnetometerMsg.orthogonalstrength[2];

            udpSenderMagnetometer.Send(&magnetometer,sizeof(magnetometer));
        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
