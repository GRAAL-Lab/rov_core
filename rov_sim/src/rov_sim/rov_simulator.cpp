//#include <cmath>
//#include <iomanip>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

//#include "GeographicLib/UTMUPS.hpp"
#include "rov_msgs/topicnames.hpp"

#include "rov_sim/rov_simulator.hpp"
#include "rov_sim/simulator_defines.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "rov_msgs/topicnames.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rov {
using namespace std::chrono_literals;
using std::placeholders::_1;

VehicleSimulator::VehicleSimulator(const std::string file_name)
    : Node("simulator_node1")
    , geod_(GeographicLib::Geodesic::WGS84())
    , gpsPubCounter_(0)
    , compassPubCounter_(0)
    , imuPubCounter_(0)
    , magnetometerPubCounter_(0)
    , pressurePubConter_(0)
    //, ambientPubCounter_(0)
    //, orientusPubCounter_(0)
    //, dvlPubCounter_(0)
    //, fogPubCounter_(0)
    , realTime_(false) // we set a manual set sample time later
{

    config_ = std::make_shared<rov::SimulatorConfiguration>();

    if (!LoadConfiguration(file_name)) {
        exit(EXIT_FAILURE);
    }

    rovModel_.params = config_->ROVmodelParams;
    rovModel_.Cable_params = config_->CableROVmodelParams;
    //std::cout << config_->modelParams << std::endl;

    // setting initial location of the ROV
    std::cout << "centroid" << centroidLocation_ << std::endl;
    vehiclePos_ = vehiclePreviousPos_ = centroidLocation_;
    altitude_ = Pre_altitude_ = 0.0;
    previous_bodyF_orientation_.Roll(0.0); bodyF_orientation_.Roll(0.0);
    previous_bodyF_orientation_.Pitch(0.0); bodyF_orientation_.Pitch(0.0);
    previous_bodyF_orientation_.Yaw(0.0); bodyF_orientation_.Yaw(0.0);
    std::cout << "INITIAL POS: LatLongAlt = " << vehiclePos_.latitude << ", " << vehiclePos_.longitude<< ", " << altitude_ << "\n";


    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();
    SetSampleTime(0.0103); // delta time taken from a simulation interval

    microLoopCountPub_ = this->create_publisher<rov_msgs::msg::MicroLoopCount>(rov_msgs::topicnames::micro_loop_count, 1);
    gpsPub_ = this->create_publisher<rov_msgs::msg::GPSData>(rov_msgs::topicnames::sensor_gps_data, 1);
    compassPub_ = this->create_publisher<rov_msgs::msg::Compass>(rov_msgs::topicnames::sensor_compass, 1);
    imuPub_ = this->create_publisher<rov_msgs::msg::IMUData>(rov_msgs::topicnames::sensor_imu, 1);
    //ambsensPub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient, 1);
    magnetometerPub_ = this->create_publisher<rov_msgs::msg::Magnetometer>(rov_msgs::topicnames::sensor_magnetometer, 1);
    pressurePub_ = this->create_publisher<rov_msgs::msg::PressureData>(rov_msgs::topicnames::sensor_pressure, 1);
    //dvlPub_ = this->create_publisher<ulisse_msgs::msg::DVLData>(ulisse_msgs::topicnames::sensor_dvl, 1);
    //fogPub_ = this->create_publisher<ulisse_msgs::msg::FOGData>(ulisse_msgs::topicnames::sensor_fog, 1);
    //appliedMotorRefPub_ = this->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_applied_perc, 1);
    simulatedSystemPub_ = this->create_publisher<rov_msgs::msg::SimulatedSystem>(rov_msgs::topicnames::simulated_system, 1);
    forcesPub_ = this->create_publisher<rov_msgs::msg::Forces>(rov_msgs::topicnames::forces, 1);
    //tfPub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(rov_msgs::topicnames::tf, 1);
    posePub_= this->create_publisher<geometry_msgs::msg::PoseStamped>(rov_msgs::topicnames::posROV, 1);
    //tf_static_broadcaster_ = this->create_publisher<tf2_ros::StaticTransformBroadcaster> (rov_msgs::topicnames::tf, 1);
    //tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    //tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_broadcaster_ROV = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //srvUserInput_ = this->create_service<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service,
    //                                                               std::bind(&VehicleSimulator::CommandsHandler, this, 1));
    //srvUserInput_ = this->create_service<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service, std::bind(&VehicleSimulator::CommandsHandler, this, _1, _2, _3));

    //motorsDataPub_ = this->create_publisher<ulisse_msgs::msg::LLCThrusters>(ulisse_msgs::topicnames::llc_thrusters, 1);

    thrustersSub_ = this->create_subscription<rov_msgs::msg::ThrustersReference>(rov_msgs::topicnames::llc_thrusters_reference_perc, 1,
        std::bind(&VehicleSimulator::ThrustersReferenceCB, this, _1));

    worldF_waterVelocity_(0) = 0.0;
    worldF_waterVelocity_(1) = 0.0;
    worldF_waterVelocity_(2) = 0.0;

    n_p_ = 0;
    n_s_ = 0;

    bodyF_projection_.setZero(6, 6);

    // Main function timer
    int msRunPeriod = 1.0 / (config_->rate) * 1000;
    // std::cout << "Controller Rate: " << rate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&VehicleSimulator::Run, this));
    std::cout << "set time " << "\n";

    Eigen::Vector6d eta_initial, vel_initial;
    Eigen::Vector3d pos_initial;
    vel_initial.setZero();

    std::cout << "centroid_" << centroid_ << std::endl;
    ctb::LatLong2LocalUTM(vehiclePos_, altitude_, centroidLocation_, pos_initial);
    //ctb::LatLong2LocalUTM(eta_initial.segment(0,3), centroid_, startP_, altitude_);
    std::cout << "pos_init" << pos_initial << std::endl;

    // Initializing vectors and matrices
    eta_initial.setZero();
    eta_initial.segment(0,3) = pos_initial;
    //rovModel_.InitializeMatrices(vel_initial, eta_initial);

    ROVpose_ = ROVprepose_ = pos_initial;

    if(rovModel_.params.heavyConf)
        volt_cmd.resize(8,1);
    else
        volt_cmd.resize(6,1);
    volt_cmd.setZero(); // volt should be between 0 to 1
    //volt_cmd[0] = 0.05;
    //volt_cmd[1] = 0.05;
    //volt_cmd[2] = 0.02;
    //volt_cmd[3] = -0.02;
    //volt_cmd[4] = -0.02;
    //volt_cmd[5] = 0.02;

    // temp_frame in Rviz visualization
    t_stamp_temp.transform.translation.x = 0;
    t_stamp_temp.transform.translation.y = 0;
    t_stamp_temp.transform.translation.z = 0;

    //pos_initial.x()= pos_initial.x() + 1.0;

    // setting initial location of cable ends <world_F>


    Eigen::RotationMatrix Rz, Ry, Rx;
    Rz << cos(bodyF_orientation_.Yaw()), -sin(bodyF_orientation_.Yaw()), 0,
        sin(bodyF_orientation_.Yaw()), cos(bodyF_orientation_.Yaw()), 0,
        0, 0, 1;

    Ry << cos(bodyF_orientation_.Pitch()), 0, sin(bodyF_orientation_.Pitch()),
        0, 1, 0,
        -sin(bodyF_orientation_.Pitch()), 0, cos(bodyF_orientation_.Pitch());

    Rx << 1, 0, 0,
        0, cos(bodyF_orientation_.Roll()), -sin(bodyF_orientation_.Roll()),
        0, sin(bodyF_orientation_.Roll()), cos(bodyF_orientation_.Roll());

    worldF_R_bodyF_ = Rz * Ry * Rx;
    //Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF_.transpose();

    rovModel_.InitializeMatrices(vel_initial, worldF_R_bodyF_);

    bodyF_cable_ending_ = { -rovModel_.params.L / 2, 0.0, 0.0};
    //bodyF_cable_ending_ = { 1.0, 0.0, 0.0};
    Eigen::Vector3d worldF_cable_ending;
    worldF_cable_ending =  worldF_R_bodyF_ * bodyF_cable_ending_;
    worldF_cable_ending =  worldF_cable_ending + pos_initial;
    ctb::LocalUTM2LatLong(worldF_cable_ending, centroidLocation_, cableEndPos_, cableEnd_altitude_);
    rovModel_.SetCableLength(4.0);

    Eigen::Vector3d worldF_cable_starting;
    ctb::LatLong cable_starting_, cable_ending_;
    worldF_cable_starting = {-1.0, 0, 0};
    ctb::LocalUTM2LatLong(worldF_cable_starting, centroidLocation_, cableStartPos_, cableStart_altitude_);

    //std::cout << "Please Enter a command for ROV motion: " << std::endl;
    std::cout << "Motion type : hold" << std::endl;
    option = rov::inputs::ID::hold;
    //
    //getchar();
}

bool VehicleSimulator::LoadConfiguration(const std::string file_name)
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
    confPath = (ament_index_cpp::get_package_share_directory("rov_sim")).append("/conf/").append(file_name);

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

void VehicleSimulator::SetRealtime(bool realtime)
{
    realTime_ = realtime;
}

void VehicleSimulator::SetSampleTime(double ts)
{
    Ts_fixed_ = ts;
}

void VehicleSimulator::Run()
{
    //getchar();
    ExecuteStep();
    SimulateSensors();
    PublishSensors();
}


void VehicleSimulator::ExecuteStep()
{
    // We reset the motor reference in case we don't receive any message for more than one second
    if (motorTimeout_.Elapsed() > 1.0) {
        hp_ = hs_ = 0.0;
    }

    std::clamp(hp_, -100.0, 100.0);
    std::clamp(hs_, -100.0, 100.0);

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

    SimulateActuation();

    t_last_ = t_now_;
    previous_bodyF_orientation_ = bodyF_orientation_;
    vehiclePreviousPos_ = vehiclePos_;
    Pre_altitude_ = altitude_;

    ROVprepose_ = ROVpose_;
}

void VehicleSimulator::SimulateActuation()
{
    Eigen::Vector6d vehicle_eta;
    Eigen::Vector3d vehicle_pos;

    // Convert from LatLong to Local UTM for computation
    Eigen::Vector3d cableS_pos_worldF;
    ctb::LatLong2LocalUTM(cableStartPos_, cableStart_altitude_, centroidLocation_, cableS_pos_worldF);
    Eigen::Vector3d cableE_pos_worldF;
    ctb::LatLong2LocalUTM(cableEndPos_, cableEnd_altitude_, centroidLocation_, cableE_pos_worldF);

    ctb::LatLong2LocalUTM(vehiclePos_, altitude_, centroidLocation_, vehicle_pos);
    vehicle_eta.segment(0,3) = vehicle_pos;
    vehicle_eta(3) = bodyF_orientation_.Roll();
    vehicle_eta(4) = bodyF_orientation_.Pitch();
    vehicle_eta(5) = bodyF_orientation_.Yaw();

    // Computing rov acceleration
    Eigen::Vector6d bodyF_cableForce;
    float cable_length = rovModel_.GetCableCurrentLength();
    //Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF_.transpose();

    bodyF_cableForce = rovModel_.ComputeFcable_bodyF(cableS_pos_worldF, cableE_pos_worldF, cable_length, worldF_R_bodyF_);

    //std::cout << "volt_cmd = "<< volt_cmd << std::endl;
    rovModel_.DirectDynamics(volt_cmd, bodyF_cableForce, worldF_R_bodyF_, bodyF_relativeVelocity_, bodyF_relativeAcceleration_);

    Eigen::RotationMatrix Rz, Ry, Rx;
    Rz << cos(bodyF_orientation_.Yaw()), -sin(bodyF_orientation_.Yaw()), 0,
        sin(bodyF_orientation_.Yaw()), cos(bodyF_orientation_.Yaw()), 0,
        0, 0, 1;

    Ry << cos(bodyF_orientation_.Pitch()), 0, sin(bodyF_orientation_.Pitch()),
        0, 1, 0,
        -sin(bodyF_orientation_.Pitch()), 0, cos(bodyF_orientation_.Pitch());

    Rx << 1, 0, 0,
        0, cos(bodyF_orientation_.Roll()), -sin(bodyF_orientation_.Roll()),
        0, sin(bodyF_orientation_.Roll()), cos(bodyF_orientation_.Roll());

    worldF_R_bodyF_ = Rz * Ry * Rx;

    // Compute the projection of the velocity on the plane (non "vola")
    //Eigen::Vector3d worldF_wFk = { 0.0, 0.0, 1.0 };

    //bodyF_wFk_ = worldF_R_bodyF_.transpose() * worldF_wFk;

    //P_ = Eigen::Matrix3d::Identity() - bodyF_wFk_ * bodyF_wFk_.transpose();

    //bodyF_projection_.block(0, 0, 3, 3) = P_;
    //bodyF_projection_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

    //bodyF_relativeAcceleration_projected_ = bodyF_projection_ * bodyF_relativeAcceleration_;

    /*
    worldF_waterVelocity_(0) = config_->inertialF_waterCurrent.x();
    worldF_waterVelocity_(1) = config_->inertialF_waterCurrent.y();
    worldF_waterVelocity_(2) = config_->inertialF_waterCurrent.z(); // for ROV
*/
    worldF_waterVelocity_(0) = 0.0;
    worldF_waterVelocity_(1) = 0.0;
    worldF_waterVelocity_(2) = 0.0;

    // Integrating the acceleration to get the vehicle velocity
    //bodyF_relativeVelocity_ = bodyF_relativeVelocity_ + bodyF_relativeAcceleration_projected_ * Ts_;
    bodyF_relativeVelocity_ = bodyF_relativeVelocity_ + bodyF_relativeAcceleration_ * Ts_;

    // Projecting the acceleration and velocity on the world frame
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
    double t = now_stamp_secs + (now_stamp_nanosecs * 1e-9);

    bodyF_wavesEffects_ <<
        0.0,
        0.0,
        0.0,
        config_->wx.A * sin(2 * M_PI * config_->wx.f * t) + config_->wx.C,
        config_->wy.A * sin(2 * M_PI * config_->wy.f * t) + config_->wy.C,
        config_->wz.A * sin(2 * M_PI * config_->wz.f * t) + config_->wz.C; // 3d ROV motion

    worldF_relativeAcceleration_ = bodyF_orientation_.ToRotationMatrix().CartesianRotationMatrix() * bodyF_relativeAcceleration_;
    worldF_relativeVelocity_ = bodyF_orientation_.ToRotationMatrix().CartesianRotationMatrix() * (bodyF_relativeVelocity_ + bodyF_wavesEffects_);

    // Get the vehicle absolute velocity by adding the water current velocity
    worldF_velocity_ = worldF_relativeVelocity_ + worldF_waterVelocity_;

    // Passing from angular vehicle velocity to Euler rates
    Eigen::Matrix3d S;
    S << cos(bodyF_orientation_.Yaw()) * cos(bodyF_orientation_.Pitch()), -sin(bodyF_orientation_.Yaw()), 0,
        sin(bodyF_orientation_.Yaw()) * cos(bodyF_orientation_.Pitch()), cos(bodyF_orientation_.Yaw()), 0,
        -sin(bodyF_orientation_.Pitch()), 0, 1;
    rml::RegularizationData mySvd;
    Eigen::Vector3d rpyEulerRates = rml::RegularizedPseudoInverse(S, mySvd) * worldF_velocity_.AngularVector();

    // Integrating the linear velocity to get the new position
    vehicleTrack_ = std::atan2(worldF_velocity_(1), worldF_velocity_(0));

    vehicleTrack_ = std::fmod(vehicleTrack_ + 2 * M_PI, 2 * M_PI);

    vehicleSpeed_ = std::sqrt(worldF_velocity_(0) * worldF_velocity_(0) + worldF_velocity_(1) * worldF_velocity_(1));
    double distance_ = vehicleSpeed_ * Ts_;

    geod_.Direct(vehiclePreviousPos_.latitude, vehiclePreviousPos_.longitude, vehicleTrack_ * 180.0 / M_PI, distance_, vehiclePos_.latitude, vehiclePos_.longitude);

    altitude_ = Pre_altitude_ + worldF_velocity_(2) * Ts_;

    ctb::LatLong2LocalNED(vehiclePos_, altitude_, centroidLocation_, ROVpose_);
    //ROVpose_(0) = ROVprepose_(0) + worldF_velocity_(0) * Ts_;
    //ROVpose_(1) = ROVprepose_(1) + worldF_velocity_(1) * Ts_;
    /*
    ROVpose_(0) = ROVprepose_(0) + worldF_velocity_(0) * Ts_;
    ROVpose_(1) = ROVprepose_(1) + worldF_velocity_(1) * Ts_;
    ROVpose_(2) = ROVprepose_(2) + worldF_velocity_(2) * Ts_;
    */

    if (altitude_ < 0) altitude_ = 0;
    ROVpose_(2) = altitude_;

    // Integrating the Euler rates to get the new Euler angles and wrapping around 2 PI
    bodyF_orientation_.Roll(std::fmod((previous_bodyF_orientation_.Roll() + rpyEulerRates(0) * Ts_) + 2 * M_PI, 2 * M_PI));
    bodyF_orientation_.Pitch(std::fmod((previous_bodyF_orientation_.Pitch() + rpyEulerRates(1) * Ts_) + 2 * M_PI, 2 * M_PI));
    bodyF_orientation_.Yaw(std::fmod((previous_bodyF_orientation_.Yaw() + rpyEulerRates(2) * Ts_) + 2 * M_PI, 2 * M_PI));

    // update cable ending pos
    Eigen::Vector3d worldF_cable_ending;
    worldF_cable_ending =  worldF_R_bodyF_ * bodyF_cable_ending_;
    worldF_cable_ending =  worldF_cable_ending + ROVpose_;
    ctb::LocalUTM2LatLong(worldF_cable_ending, centroidLocation_, cableEndPos_, cableEnd_altitude_);
    cableEndPosXY_ = worldF_cable_ending;
}

void VehicleSimulator::SimulateSensors()
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

    auto elapsed_secs = static_cast<double>(total_elapsed_.count()) / 1E9;
    timestamp_count_ = static_cast<uint32_t>(elapsed_secs * 200.0);
    stepssincepps_count_ = static_cast<uint32_t>(elapsed_secs * 200.0) % 200;

    microLoopCountMsg_.timestamp = timestamp_count_;
    microLoopCountMsg_.stepssincepps = stepssincepps_count_;

    // construct a trivial random generator engine from a time-based seed:
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    /////   GPS   /////
    std::normal_distribution<double> gpsNoiseX(0.0, config_->sensorsNoise.gps_stdd.x());
    std::normal_distribution<double> gpsNoiseY(0.0, config_->sensorsNoise.gps_stdd.y());
    std::normal_distribution<double> gpsNoiseZ(0.0, config_->sensorsNoise.gps_stdd.z());

    //Transform to cartesian
    Eigen::Vector3d worldF_com, worldF_antenna;
    ctb::LatLong2LocalNED(vehiclePos_, altitude_, centroidLocation_, worldF_com);

    //move the gps from COM to the antenna
    worldF_antenna = worldF_com + worldF_R_bodyF_ * config_->bodyF_gps_sensor_position;

    //add noise and come back to map coordinates
    worldF_antenna.x() += gpsNoiseX(generator);
    worldF_antenna.y() += gpsNoiseY(generator);
    worldF_antenna.z() += gpsNoiseZ(generator);

    ctb::LatLong gpsLatlong;
    double gpsAltitude;
    ctb::LocalNED2LatLong(worldF_antenna, centroidLocation_, gpsLatlong, gpsAltitude);

    gpsMsg_.time = static_cast<double>(now_nanosecs / 1E9);
    gpsMsg_.track = vehicleTrack_;
    gpsMsg_.speed = vehicleSpeed_;
    gpsMsg_.latitude = gpsLatlong.latitude;
    gpsMsg_.longitude = gpsLatlong.longitude;
    gpsMsg_.altitude = gpsAltitude;
    gpsMsg_.gpsfixmode = rov_msgs::msg::GPSData::MODE3D; //3u


    /////   COMPASS   /////
    std::normal_distribution<double> compassNoiseR(0.0, config_->sensorsNoise.compass_stdd.x());
    std::normal_distribution<double> compassNoiseP(0.0, config_->sensorsNoise.compass_stdd.y());
    std::normal_distribution<double> compassNoiseY(0.0, config_->sensorsNoise.compass_stdd.z());

    compassMsg_.stamp.sec = now_stamp_secs;
    compassMsg_.stamp.nanosec = now_stamp_nanosecs;
    compassMsg_.orientation.roll = bodyF_orientation_.Roll() + compassNoiseR(generator);
    compassMsg_.orientation.pitch = bodyF_orientation_.Pitch() + compassNoiseP(generator);
    compassMsg_.orientation.yaw = bodyF_orientation_.Yaw() + compassNoiseY(generator);


    /////   MAGNETOMETER   /////
    Eigen::Vector3d m = { 23186.6 * 1E-9, 0.0 * 1E-9, 41122.0 * 1E-9 };  // Example of magnetic field at lat long: 44.4056° N, 8.9463° E

    Eigen::Vector3d ned_m = worldF_R_bodyF_.transpose() * m;

    std::normal_distribution<double> magnetometerNoiseX(0.0, config_->sensorsNoise.magnetometer_stdd.x());
    std::normal_distribution<double> magnetometerNoiseY(0.0, config_->sensorsNoise.magnetometer_stdd.y());
    std::normal_distribution<double> magnetometerNoiseZ(0.0, config_->sensorsNoise.magnetometer_stdd.z());

    magnetometerMsg_.stamp.sec = now_stamp_secs;
    magnetometerMsg_.stamp.nanosec = now_stamp_nanosecs;
    magnetometerMsg_.orthogonalstrength[0] = ned_m.x() + magnetometerNoiseX(generator);
    magnetometerMsg_.orthogonalstrength[1] = ned_m.y() + magnetometerNoiseY(generator);
    magnetometerMsg_.orthogonalstrength[2] = ned_m.z() + magnetometerNoiseZ(generator);


    /////   IMU   /////
    /// Imu: Orientation
    std::normal_distribution<double> orientusNoiseX(0.0, config_->sensorsNoise.orientus_stdd.x());
    std::normal_distribution<double> orientusNoiseY(0.0, config_->sensorsNoise.orientus_stdd.y());
    std::normal_distribution<double> orientusNoiseZ(0.0, config_->sensorsNoise.orientus_stdd.z());

    imuMsg_.stamp.sec = now_stamp_secs;
    imuMsg_.stamp.nanosec = now_stamp_nanosecs;
    Eigen::RotationMatrix bodyF_R_orientus = rml::EulerRPY(config_->bodyF_imu_sensor_pose.AngularVector()).ToRotationMatrix();
    rml::EulerRPY imuF_orientation = Eigen::RotationMatrix(bodyF_orientation_.ToRotationMatrix() * bodyF_R_orientus).ToEulerRPY();
    imuMsg_.orientation.roll = imuF_orientation.Roll() + compassNoiseR(generator);
    imuMsg_.orientation.pitch = imuF_orientation.Pitch() + compassNoiseP(generator);
    imuMsg_.orientation.yaw = imuF_orientation.Yaw() + compassNoiseY(generator);


    /// Imu: Linear Velocity
    ///
    ///
    /// Imu: Angular Velocity
    ///
    ///
    /// Imu: Linear Acceleration
    ///
    ///
    /// Imu: Angular Acceleration
    ///
    ///
    /// Imu: Accelerometer (raw)
    std::normal_distribution<double> accelerometerNoiseX(0.0, config_->sensorsNoise.accelerometer_stdd.x());
    std::normal_distribution<double> accelerometerNoiseY(0.0, config_->sensorsNoise.accelerometer_stdd.y());
    std::normal_distribution<double> accelerometerNoiseZ(0.0, config_->sensorsNoise.accelerometer_stdd.z());

    imuMsg_.stamp.sec = now_stamp_secs;
    imuMsg_.stamp.nanosec = now_stamp_nanosecs;
    Eigen::Vector6d bodyF_relativeAcceleration;
    Eigen::Vector3d worldF_gravity = { 0.0, 0.0, -9.81 };
    bodyF_relativeAcceleration.segment(0, 3) = bodyF_relativeAcceleration_projected_.segment(0, 3) + worldF_R_bodyF_.transpose() * worldF_gravity;

    // !!!!Matrice di corpo rigido per le accelerazioni? (TODO -> RICERCA/RICAVA FORMULA)
    // !Eigen::Matrix6d bodyF_RBM_imu = config_->bodyF_imu_sensor_pose.LinearVector().GetRigidBodyMatrix();
    // !Eigen::Vector6d bodyF_relativeAcceleration_imu = bodyF_RBM_imu * bodyF_relativeAcceleration;
    Eigen::RotationMatrix bodyF_R_imu = rml::EulerRPY(config_->bodyF_imu_sensor_pose.AngularVector()).ToRotationMatrix();
    Eigen::Vector3d imuF_relativeLinearAcceleration = bodyF_R_imu.transpose() * bodyF_relativeAcceleration.LinearVector();

    imuMsg_.accelerometer[0] = imuF_relativeLinearAcceleration.x() + accelerometerNoiseX(generator);
    imuMsg_.accelerometer[1] = imuF_relativeLinearAcceleration.y() + accelerometerNoiseY(generator);
    imuMsg_.accelerometer[2] = imuF_relativeLinearAcceleration.z() + accelerometerNoiseZ(generator);


    /// Imu: Gyroscope (raw)
    std::normal_distribution<double> gyroNoiseX(0.0, config_->sensorsNoise.gyro_stdd.x());
    std::normal_distribution<double> gyroNoiseY(0.0, config_->sensorsNoise.gyro_stdd.y());
    std::normal_distribution<double> gyroNoiseZ(0.0, config_->sensorsNoise.gyro_stdd.z());

    //gyro bias model as a very low frequence sin + const
    double t = now_stamp_secs + (now_stamp_nanosecs * 1e-9);
    double bx = config_->sensorsNoise.bx.C + config_->sensorsNoise.bx.A * sin(2 * M_PI * config_->sensorsNoise.bx.f * t);
    double by = config_->sensorsNoise.by.C + config_->sensorsNoise.by.A * sin(2 * M_PI * config_->sensorsNoise.by.f * t);
    double bz = config_->sensorsNoise.bz.C + config_->sensorsNoise.bz.A * sin(2 * M_PI * config_->sensorsNoise.bz.f * t);

    //add sin wave to simulate the effects of water waves
    Eigen::Vector3d bodyF_relativeAngularVelocity;
    bodyF_relativeAngularVelocity(0) = bodyF_relativeVelocity_(3) + bodyF_wavesEffects_(3);
    bodyF_relativeAngularVelocity(1) = bodyF_relativeVelocity_(4) + bodyF_wavesEffects_(4);
    bodyF_relativeAngularVelocity(2) = bodyF_relativeVelocity_(5);

    imuMsg_.gyro[0] = bodyF_relativeAngularVelocity(0) + gyroNoiseX(generator) + bx;
    imuMsg_.gyro[1] = bodyF_relativeAngularVelocity(1) + gyroNoiseY(generator) + by;
    imuMsg_.gyro[2] = bodyF_relativeAngularVelocity(2) + gyroNoiseZ(generator) + bz;

    /// Imu: Magnetometer
    ///
    ///

    /////   PRESSURE SENSOR   /////
    std::normal_distribution<double> pressureNoiseR(0.0, config_->sensorsNoise.compass_stdd.x());
    pressureMsg_.stamp.sec = now_stamp_secs;
    pressureMsg_.stamp.nanosec = now_stamp_nanosecs;
    pressureMsg_.depth = altitude_ + pressureNoiseR(generator);

    // Fill the ground truth msg
    groundTruthMsg_.stamp.sec = now_stamp_secs;
    groundTruthMsg_.stamp.nanosec = now_stamp_nanosecs;
    groundTruthMsg_.inertialframe_linear_position.latlong.latitude = vehiclePos_.latitude;
    groundTruthMsg_.inertialframe_linear_position.latlong.longitude = vehiclePos_.longitude;
    groundTruthMsg_.inertialframe_linear_position.altitude = altitude_;
    groundTruthMsg_.bodyframe_angular_position.roll = bodyF_orientation_.Roll();
    groundTruthMsg_.bodyframe_angular_position.pitch = bodyF_orientation_.Pitch();
    groundTruthMsg_.bodyframe_angular_position.yaw = bodyF_orientation_.Yaw();
    groundTruthMsg_.bodyframe_linear_velocity[0] = bodyF_relativeVelocity_(0);
    groundTruthMsg_.bodyframe_linear_velocity[1] = bodyF_relativeVelocity_(1);
    groundTruthMsg_.bodyframe_linear_velocity[2] = bodyF_relativeVelocity_(2);
    groundTruthMsg_.bodyframe_angular_velocity[0] = bodyF_relativeAngularVelocity(0);
    groundTruthMsg_.bodyframe_angular_velocity[1] = bodyF_relativeAngularVelocity(1);
    groundTruthMsg_.bodyframe_angular_velocity[2] = bodyF_relativeAngularVelocity(2);
    groundTruthMsg_.inertialframe_water_current[0] = worldF_waterVelocity_[0];
    groundTruthMsg_.inertialframe_water_current[1] = worldF_waterVelocity_[1]; // worldF_waterVelocity_[2]??
    groundTruthMsg_.inertialframe_water_current[2] = worldF_waterVelocity_[2];
    groundTruthMsg_.gyro_bias[0] = bx;
    groundTruthMsg_.gyro_bias[1] = by;
    groundTruthMsg_.gyro_bias[2] = bz;

    //std::cout<< "vehiclePos_ " << vehiclePos_<< " altitude "<<altitude_<< std::endl;

    forcesMsg_.stamp.sec = now_stamp_secs;
    forcesMsg_.stamp.nanosec = now_stamp_nanosecs;
    AssignMessage(forcesMsg_.bodyframe_coriolis_drag, rovModel_.getCoriolisAndDrag_bodyF());
    AssignMessage(forcesMsg_.bodyframe_f_cable, rovModel_.getFcable_bodyF());
    AssignMessage(forcesMsg_.bodyframe_f_thruster, rovModel_.getFthruster_bodyF());
    AssignMessage(forcesMsg_.bodyframe_g, rovModel_.getg_bodyF());

    Eigen::Vector3d LaSpezia_centroid;
    ctb::LatLong2LocalUTM(centroidLocation_, 0.0, centroidLocation_, LaSpezia_centroid);

    //Eigen::Vector3d vehicle_pos;
    //ctb::LatLong2LocalUTM(vehiclePos_, altitude_, centroidLocation_, vehicle_pos);
    //tf2::Quaternion q;
    //Eigen::Quaterniond eq(worldF_R_bodyF_);

    //q.setEuler( bodyF_orientation_.Pitch(), bodyF_orientation_.Roll(), bodyF_orientation_.Yaw()); // i used it

    t_stamp.header.stamp = this->get_clock()->now();
    t_stamp.header.frame_id = "world";
    t_stamp.child_frame_id = "centroid";
    t_stamp.transform.translation.x = LaSpezia_centroid(0);
    t_stamp.transform.translation.y = LaSpezia_centroid(1);
    t_stamp.transform.translation.z = LaSpezia_centroid(2);
    t_stamp.transform.rotation.x = 1.0;
    t_stamp.transform.rotation.y = 0.0;
    t_stamp.transform.rotation.z = 0.0;
    t_stamp.transform.rotation.w = 0.0;
    tf_broadcaster_->sendTransform(t_stamp);


    //Eigen::Quaterniond eq1(worldF_R_bodyF_);
    tf2::Quaternion q2;
    q2.setRPY(bodyF_orientation_.Roll(),bodyF_orientation_.Pitch(),bodyF_orientation_.Yaw());
    //tf2::quaternionEigenToTF(eq1, q1);
    //transform.setRotation(q);
    t_stamp_ROV.transform.rotation.x = q2.x();
    t_stamp_ROV.transform.rotation.y = q2.y();
    t_stamp_ROV.transform.rotation.z = q2.z();
    t_stamp_ROV.transform.rotation.w = q2.w();

    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);

    t_stamp_ROV.header.stamp = this->get_clock()->now();
    t_stamp_ROV.header.frame_id = "world";
    t_stamp_ROV.child_frame_id = "ROV";
    t_stamp_ROV.transform.translation.x = ROVpose_.x();
    t_stamp_ROV.transform.translation.y = ROVpose_.y();
    t_stamp_ROV.transform.translation.z = altitude_;
    t_stamp_ROV.transform.rotation.x = q2.x();
    t_stamp_ROV.transform.rotation.y = q2.y();
    t_stamp_ROV.transform.rotation.z = q2.z();
    t_stamp_ROV.transform.rotation.w = q2.w();
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);

    pt_.header.stamp = this->get_clock()->now();
    pt_.header.frame_id = "ROVframe";
    pt_.pose.position.x = ROVpose_.x();
    pt_.pose.position.y = ROVpose_.y();
    pt_.pose.position.z = altitude_;
    pt_.pose.orientation.x = q2.x();
    pt_.pose.orientation.y = q2.y();
    pt_.pose.orientation.z = q2.z();
    pt_.pose.orientation.w = q2.w();

    /*t_stamp_ROV.header.stamp = this->get_clock()->now();
    t_stamp_ROV.header.frame_id = "world";
    t_stamp_ROV.child_frame_id = "cable_fixed";
    t_stamp_ROV.transform.translation.x = 1.0;
    t_stamp_ROV.transform.translation.y = 1.0;
    t_stamp_ROV.transform.translation.z = 0.0;
    t_stamp_ROV.transform.rotation.x = 1.0;
    t_stamp_ROV.transform.rotation.y = 0.0;
    t_stamp_ROV.transform.rotation.z = 0.0;
    t_stamp_ROV.transform.rotation.w = 0.0;
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);

    t_stamp_ROV.header.stamp = this->get_clock()->now();
    t_stamp_ROV.header.frame_id = "ROV";
    t_stamp_ROV.child_frame_id = "cable_ROV";
    t_stamp_ROV.transform.translation.x = -0.4;
    t_stamp_ROV.transform.translation.y = 0.0;
    t_stamp_ROV.transform.translation.z = 0.0;
    t_stamp_ROV.transform.rotation.x = 0.0;
    t_stamp_ROV.transform.rotation.y = 0.0;
    t_stamp_ROV.transform.rotation.z = 0.0;
    t_stamp_ROV.transform.rotation.w = 1.0; // 0 or 1
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);*/

    Eigen::Vector3d cableS_pos;
    ctb::LatLong2LocalUTM(cableStartPos_, cableStart_altitude_, centroidLocation_, cableS_pos);
    Eigen::Vector3d cableE_pos;
    ctb::LatLong2LocalUTM(cableEndPos_, cableEnd_altitude_, centroidLocation_, cableE_pos);

    t_stamp_ROV.header.stamp = this->get_clock()->now();
    t_stamp_ROV.header.frame_id = "world";
    t_stamp_ROV.child_frame_id = "cable_p1";
    t_stamp_ROV.transform.translation.x = cableS_pos.x();
    t_stamp_ROV.transform.translation.y = cableS_pos.y();
    t_stamp_ROV.transform.translation.z = cableS_pos.z();
    t_stamp_ROV.transform.rotation.x = 1.0;
    t_stamp_ROV.transform.rotation.y = 0.0;
    t_stamp_ROV.transform.rotation.z = 0.0;
    t_stamp_ROV.transform.rotation.w = 0.0;
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);

    t_stamp_ROV.header.stamp = this->get_clock()->now();
    t_stamp_ROV.header.frame_id = "world";
    t_stamp_ROV.child_frame_id = "cable";
    t_stamp_ROV.transform.translation.x = cableEndPosXY_.x();
    t_stamp_ROV.transform.translation.y = cableEndPosXY_.y();
    t_stamp_ROV.transform.translation.z = cableEndPosXY_.z();
    t_stamp_ROV.transform.rotation.x = q2.x();
    t_stamp_ROV.transform.rotation.y = q2.y();
    t_stamp_ROV.transform.rotation.z = q2.z();
    t_stamp_ROV.transform.rotation.w = q2.w();
    tf_broadcaster_ROV->sendTransform(t_stamp_ROV);

}

void VehicleSimulator::AssignMessage(std::array<double,6>& msg,const Eigen::Vector6d& vector){
    for(int i=0; i < msg.size(); i++){
        msg[i] = vector(i);
    }
}

void VehicleSimulator::PublishSensors()
{
    microLoopCountPub_->publish(microLoopCountMsg_);
    simulatedSystemPub_->publish(groundTruthMsg_);
    forcesPub_->publish(forcesMsg_);

    //tfPub_->publish(tt_);
    posePub_->publish(pt_);


    //appliedMotorRefPub_->publish(appliedMotorRefMsg_);
    //motorsDataPub_->publish(motorsDataMsg_);

    if (static_cast<int>(timestamp_count_ / 20) > gpsPubCounter_) {
        gpsPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        gpsPub_->publish(gpsMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > imuPubCounter_) {
        imuPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        imuPub_->publish(imuMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > compassPubCounter_) {
        compassPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        compassPub_->publish(compassMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > magnetometerPubCounter_) {
        magnetometerPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        magnetometerPub_->publish(magnetometerMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > pressurePubConter_) {
        pressurePubConter_ = static_cast<int>(timestamp_count_ / 20);
        pressurePub_->publish(pressureMsg_);
    }
    /*

    if (static_cast<int>(timestamp_count_ / 20) > ambientPubCounter_) {
        ambientPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        ambsensPub_->publish(ambsensMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > dvlPubCounter_) {
        dvlPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        dvlPub_->publish(dvlMsg_);
    }
    */
}

double VehicleSimulator::GetCurrentTimeStamp() const
{
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
    return static_cast<double>(now_nanosecs / 1E9);
}


void VehicleSimulator::ThrustersReferenceCB(const rov_msgs::msg::ThrustersReference::SharedPtr msg)
{

    volt_cmd[0] = msg->first_percentage;
    volt_cmd[1] = msg->second_percentage;
    volt_cmd[2] = msg->third_percentage;
    volt_cmd[3] = msg->forth_percentage;
    volt_cmd[4] = msg->fifth_percentage;
    volt_cmd[5] = msg->sixth_percentage;

    if(rovModel_.params.heavyConf){
        volt_cmd[6] = msg->seventh_percentage;
        volt_cmd[7] = msg->eighth_percentage;
    }

    //motorTimeout_.Start();
}

}
