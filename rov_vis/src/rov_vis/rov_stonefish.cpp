//#include <cmath>
//#include <iomanip>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

//#include "GeographicLib/UTMUPS.hpp"

#include "rov_vis/rov_stonefish.hpp"
//#include "rov_sim/simulator_defines.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "rov_msgs/topicnames.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rov {
using namespace std::chrono_literals;
using std::placeholders::_1;

StoneFishROVVisualizer::StoneFishROVVisualizer(const std::string file_name)
    : Node("simulator_node1")
    
{
    config_ = std::make_shared<rov::VisualizerConfiguration>();

    if (!LoadConfiguration(file_name)) {
        exit(EXIT_FAILURE);
    }

    std::cout << "centroid" << centroidLocation_ << std::endl;
    ctb::LatLong2LocalUTM(centroidLocation_, 0.0, centroidLocation_, centerUTM_);

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();

    navDataSub_ = this->create_subscription<rov_msgs::msg::NavFilterData>(rov_msgs::topicnames::nav_filter_data, 1, std::bind(&StoneFishROVVisualizer::NavDataCB, this, _1));
    simulatedSysSub_ = this->create_subscription<rov_msgs::msg::SimulatedSystem>(rov_msgs::topicnames::simulated_system,1, std::bind(&StoneFishROVVisualizer::SimSystemCB, this, _1));
    visualizationPub_ = this->create_publisher<visualization_msgs::msg::Marker> ("visualization_marker", 0 );
    rovPosePub_ = this->create_publisher<nav_msgs::msg::Odometry> ("/sf/bluerov/pose", 1 );

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster_ROV = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    rovMarker_.header.frame_id = "NED";
    rovMarker_.scale.x = 1;
    rovMarker_.scale.y = 1;
    rovMarker_.scale.z = 1;
    rovMarker_.color.a = 1.0; // Don't forget to set the alpha!
    rovMarker_.color.r = 0.859;
    rovMarker_.color.g = 1.0;
    rovMarker_.color.b = 1.0;
    rovMarker_.ns = "rov_link";
    rovMarker_.id = 0;
    rovMarker_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    rovMarker_.mesh_resource = "package://rov_sim/meshes/BlueRov2.dae";

    // Main function timer
    int msRunPeriod = 1.0 / (config_->rate) * 1000;
    // std::cout << "Controller Rate: " << rate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&StoneFishROVVisualizer::Run, this));
    std::cout << "set time " << "\n";

}

bool StoneFishROVVisualizer::LoadConfiguration(const std::string file_name)
{
    libconfig::Config confObj;


    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD CONFIGURATION FROM NAV FILTER TO READ CENTROID
    ///
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter_rov");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter_rov.conf");

    std::cout << "PATH TO NAV_FILTER CONF FILE : " << confPath << std::endl;

    // Read the file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    //acquired the centroid location
    Eigen::VectorXd centroidLocationTmp;
    if (!ctb::GetParamVector(confObj, centroidLocationTmp, "centroidLocation")) {
        std::cerr << "Failed to load centroidLocation from file" << std::endl;
        return false;
    };

    centroidLocation_ = ctb::LatLong(centroidLocationTmp[0], centroidLocationTmp[1]);

    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD SIMULATOR CONFIGURATION
    ///
    libconfig::Config confObjSim;
    confPath = (ament_index_cpp::get_package_share_directory("rov_vis")).append("/conf/").append(file_name);

    std::cout << "PATH TO SIMULATOR CONF FILE : " << confPath << std::endl;

    try {
        confObjSim.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return -1;
    }


    if (!config_->ConfigureFromFile(confObjSim)){
        std::cerr << "Simulator node: Failed to load config params from files" << std::endl;
        return false;
    }

    return true;
}

void StoneFishROVVisualizer::Run()
{
    ExecuteStep();
    UpdateFrames();
    Visualize();
    //PublishTf();
    //PublishMarker();
}

void StoneFishROVVisualizer::ExecuteStep()
{
    // We reset the motor reference in case we don't receive any message for more than one second
    /*
    if (motorTimeout_.Elapsed() > 1.0) {
        hp_ = hs_ = 0.0;
    }

    std::clamp(hp_, -100.0, 100.0);
    std::clamp(hs_, -100.0, 100.0);
    */

    if (realTime_) {
        t_now_ = std::chrono::system_clock::now();
    } else {
        t_now_ = t_now_ + std::chrono::milliseconds(static_cast<long>(Ts_ * 1000.0));
    }

    iter_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_last_);
    total_elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_ - t_start_);

    if (realTime_) {
        Ts_ = iter_elapsed_.count() / 1E9;
    } else {
        Ts_ = Ts_fixed_;
    }

    //SimulateActuation();

    t_last_ = t_now_;
    //previous_bodyF_orientation_ = bodyF_orientation_;
    //vehiclePreviousPos_ = vehiclePos_;
    //Pre_altitude_ = altitude_;
    //ROVprepose_ = ROVpose_;
}

void StoneFishROVVisualizer::AssignMessage(std::array<double,6>& msg,const Eigen::Vector6d& vector){
    for(unsigned long i=0; i < msg.size(); i++){
        msg[i] = vector(i);
    }
}

double StoneFishROVVisualizer::GetCurrentTimeStamp() const
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    return static_cast<double>(now_nanosecs / 1E9);
}

void StoneFishROVVisualizer::PublishTf(){

    t_stamp_.header.stamp = this->get_clock()->now();
    t_stamp_.header.frame_id = "world";
    t_stamp_.child_frame_id = "NED";
    t_stamp_.transform.translation.x = centerUTM_(0);
    t_stamp_.transform.translation.y = centerUTM_(1);
    t_stamp_.transform.translation.z = centerUTM_(2);
    t_stamp_.transform.rotation.x = 1.0;
    t_stamp_.transform.rotation.y = 0.0;
    t_stamp_.transform.rotation.z = 0.0;
    t_stamp_.transform.rotation.w = 0.0;
    tf_broadcaster_->sendTransform(t_stamp_);


    //Eigen::Quaterniond eq1(worldF_ROV_bodyF_);
    //tf2::Quaternion ROVq;
    //ROVq.setRPY(navData_.bodyframe_angular_position.roll, navData_.bodyframe_angular_position.pitch, navData_.bodyframe_angular_position.yaw);

    rovSimLatLong_.latitude = simData_.inertialframe_linear_position.latlong.latitude;
    rovSimLatLong_.longitude = simData_.inertialframe_linear_position.latlong.longitude;
    ctb::LatLong2LocalUTM(rovSimLatLong_, simData_.inertialframe_linear_position.altitude, centroidLocation_, rovSimUTM_);

    //tf2::Quaternion q1;
    rovSimQ_.setRPY(simData_.bodyframe_angular_position.roll, simData_.bodyframe_angular_position.pitch, simData_.bodyframe_angular_position.yaw);

    t_stamp_ROV_.header.stamp = this->get_clock()->now();
    t_stamp_ROV_.header.frame_id = "NED";
    t_stamp_ROV_.child_frame_id = "ROV";
    t_stamp_ROV_.transform.translation.x = rovSimUTM_.y();
    t_stamp_ROV_.transform.translation.y = rovSimUTM_.x();
    t_stamp_ROV_.transform.translation.z = rovSimUTM_.z();
    t_stamp_ROV_.transform.rotation.x = rovSimQ_.x();
    t_stamp_ROV_.transform.rotation.y = rovSimQ_.y();
    t_stamp_ROV_.transform.rotation.z = rovSimQ_.z();
    t_stamp_ROV_.transform.rotation.w = rovSimQ_.w();
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV_);

}

void StoneFishROVVisualizer::PublishMarker(){

    rovNavLatLong_.latitude = navData_.inertialframe_linear_position.latlong.latitude;
    rovNavLatLong_.longitude = navData_.inertialframe_linear_position.latlong.longitude;
    ctb::LatLong2LocalUTM(rovNavLatLong_, navData_.inertialframe_linear_position.altitude, centroidLocation_, rovNavUTM_);
    rovNavQ_.setEuler(bodyF_ROV_.Yaw(),bodyF_ROV_.Pitch(),bodyF_ROV_.Roll());

    rovMarker_.header.stamp = this->get_clock()->now();
    rovMarker_.pose.position.x = rovNavUTM_.y(); // inverted
    rovMarker_.pose.position.y = rovNavUTM_.x(); // inverted
    rovMarker_.pose.position.z = navData_.inertialframe_linear_position.altitude;
    rovMarker_.pose.orientation.x = rovNavQ_.x();
    rovMarker_.pose.orientation.y = rovNavQ_.y();
    rovMarker_.pose.orientation.z = rovNavQ_.z();
    rovMarker_.pose.orientation.w = rovNavQ_.w();
    visualizationPub_->publish(rovMarker_);
}

void StoneFishROVVisualizer::UpdateFrames(){
    Eigen::RotationMatrix Rz, Ry, Rx;
    double roll, pitch, yaw;
    roll = navData_.bodyframe_angular_position.roll;
    pitch = navData_.bodyframe_angular_position.pitch;
    yaw = navData_.bodyframe_angular_position.yaw;
    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    worldF_ROV_bodyF_ = Rz * Ry * Rx;

    Eigen::RotationMatrix Rz_n;
    Rz_n << cos(-M_PI/2), -sin(-M_PI/2), 0,
        sin(-M_PI/2), cos(-M_PI/2), 0,
        0, 0, 1;
    worldF_ROV_meshF_ = Rz_n * worldF_ROV_bodyF_;
    bodyF_ROV_ = worldF_ROV_bodyF_.eulerAngles(2, 1, 0);
    //bodyF_ROV_ = worldF_ROV_meshF_.eulerAngles(2, 1, 0);
}

void StoneFishROVVisualizer::Visualize(){
    rovPose_.header.stamp = this->get_clock()->now();

    rovNavLatLong_.latitude = navData_.inertialframe_linear_position.latlong.latitude;
    rovNavLatLong_.longitude = navData_.inertialframe_linear_position.latlong.longitude;
    ctb::LatLong2LocalUTM(rovNavLatLong_, navData_.inertialframe_linear_position.altitude, centroidLocation_, rovNavUTM_);
    rovPose_.pose.pose.position.x = rovNavUTM_.y(); // inverted
    rovPose_.pose.pose.position.y = rovNavUTM_.x(); // inverted
    rovPose_.pose.pose.position.z = rovNavUTM_.z();

    tf2::Quaternion ulisse_q_;
    ulisse_q_.setEuler(bodyF_ROV_.Yaw(),bodyF_ROV_.Pitch(),bodyF_ROV_.Roll());
    rovPose_.pose.pose.orientation.x = ulisse_q_.x();
    rovPose_.pose.pose.orientation.y = ulisse_q_.y();
    rovPose_.pose.pose.orientation.z = ulisse_q_.z();
    rovPose_.pose.pose.orientation.w = ulisse_q_.w();

    rovPosePub_->publish(rovPose_);
}

void StoneFishROVVisualizer::NavDataCB(const rov_msgs::msg::NavFilterData::SharedPtr msg) { navData_ = *msg; }

void StoneFishROVVisualizer::SimSystemCB(const rov_msgs::msg::SimulatedSystem::SharedPtr msg) { simData_ = *msg; }

}
