#include <cmath>
#include <iomanip>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include "GeographicLib/UTMUPS.hpp"
#include "rov_msgs/topicnames.hpp"

#include "rov_sim/rov_simulator.hpp"

namespace rov {
using namespace std::chrono_literals;
using std::placeholders::_1;

VehicleSimulator::VehicleSimulator(const std::string file_name)
    : Node("simulator_node")
    , geod_(GeographicLib::Geodesic::WGS84())
    //, gpsPubCounter_(0)
    //, compassPubCounter_(0)
    //, imuPubCounter_(0)
    //, magnetometerPubCounter_(0)
    //, ambientPubCounter_(0)
    //, orientusPubCounter_(0)
    //, dvlPubCounter_(0)
    //, fogPubCounter_(0)
    , realTime_(true)
{

    config_ = std::make_shared<rov::SimulatorConfiguration>();

    if (!LoadConfiguration(file_name)) {
        exit(EXIT_FAILURE);
    }

    rovModel_.params = config_->modelParams;
    //std::cout << config_->modelParams << std::endl;

    vehiclePos = vehiclePreviousPos = centroidLocation;
    std::cout << "INITIAL POS: LatLong = " << vehiclePos.latitude << ", " << vehiclePos.longitude << "\n";

    t_start_ = t_last_ = t_now_ = std::chrono::system_clock::now();

    microLoopCountPub_ = this->create_publisher<rov_msgs::msg::MicroLoopCount>(rov_msgs::topicnames::micro_loop_count, 1);
    //gpsPub_ = this->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 1);
    //compassPub_ = this->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 1);
    //imuPub_ = this->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 1);
    //ambsensPub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient, 1);
    //magnetometerPub_ = this->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 1);
    //dvlPub_ = this->create_publisher<ulisse_msgs::msg::DVLData>(ulisse_msgs::topicnames::sensor_dvl, 1);
    //fogPub_ = this->create_publisher<ulisse_msgs::msg::FOGData>(ulisse_msgs::topicnames::sensor_fog, 1);
    //appliedMotorRefPub_ = this->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_applied_perc, 1);
    simulatedSystemPub_ = this->create_publisher<rov_msgs::msg::SimulatedSystem>(rov_msgs::topicnames::simulated_system, 1);
    //motorsDataPub_ = this->create_publisher<ulisse_msgs::msg::LLCThrusters>(ulisse_msgs::topicnames::llc_thrusters, 1);

    //thrustersSub_ = this->create_subscription<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc, 1,
    //    std::bind(&VehicleSimulator::ThrustersReferenceCB, this, _1));

    worldF_waterVelocity_(0) = 0.0;
    worldF_waterVelocity_(1) = 0.0;

    n_p_ = 0;
    n_s_ = 0;

    bodyF_projection_.setZero(6, 6);

    // Main function timer
    int msRunPeriod = 1.0 / (config_->rate) * 1000;
    // std::cout << "Controller Rate: " << rate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&VehicleSimulator::Run, this));

}

bool VehicleSimulator::LoadConfiguration(const std::string file_name)
{
    libconfig::Config confObj;


    ///////////////////////////////////////////////////////////////////////////////
    /////       LOAD CONFIGURATION FROM NAV FILTER TO READ CENTROID
    ///
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter.conf");

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

    centroidLocation = ctb::LatLong(centroidLocationTmp[0], centroidLocationTmp[1]);

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
    vehiclePreviousPos = vehiclePos;
    altitude_ = 0.0;
}

void VehicleSimulator::SimulateActuation()
{

    Eigen::Vector6d eta;
    eta(0) = vehiclePos.latitude;
    eta(1) = vehiclePos.longitude;
    eta(2) = altitude_;

    eta(3) = bodyF_orientation_.Yaw();
    eta(4) = bodyF_orientation_.Pitch();
    eta(5) = bodyF_orientation_.Roll();

    // Computing vehicle acceleration
    rovModel_.DirectDynamics(volt_cmd, bodyF_relativeVelocity_,eta, bodyF_relativeAcceleration_);

    //Compute the worldF_R_bodyF
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
    Eigen::Vector3d worldF_wFk = { 0.0, 0.0, 1.0 };

    bodyF_wFk_ = worldF_R_bodyF_.transpose() * worldF_wFk;

    P_ = Eigen::Matrix3d::Identity() - bodyF_wFk_ * bodyF_wFk_.transpose();

    bodyF_projection_.block(0, 0, 3, 3) = P_;
    bodyF_projection_.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

    bodyF_relativeAcceleration_projected_ = bodyF_projection_ * bodyF_relativeAcceleration_;

    worldF_waterVelocity_(0) = config_->inertialF_waterCurrent.x();
    worldF_waterVelocity_(1) = config_->inertialF_waterCurrent.y();

    // Integrating the acceleration to get the vehicle velocity
    bodyF_relativeVelocity_ = bodyF_relativeVelocity_ + bodyF_relativeAcceleration_projected_ * Ts_;

    //add projection on bodyF_relativeVelocity (plane constraint)
    bodyF_relativeVelocity_ = bodyF_projection_ * bodyF_relativeVelocity_;

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
        0.0;
    worldF_relativeAcceleration_ = bodyF_orientation_.ToRotationMatrix().CartesianRotationMatrix() * bodyF_relativeAcceleration_projected_;
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

    geod_.Direct(vehiclePreviousPos.latitude, vehiclePreviousPos.longitude, vehicleTrack_ * 180.0 / M_PI, distance_, vehiclePos.latitude, vehiclePos.longitude);

    // Integrating the Euler rates to get the new Euler angles and wrapping around PI
    bodyF_orientation_.Roll(std::fmod((previous_bodyF_orientation_.Roll() + rpyEulerRates(0) * Ts_) + 2 * M_PI, M_PI));
    bodyF_orientation_.Pitch(std::fmod((previous_bodyF_orientation_.Pitch() + rpyEulerRates(1) * Ts_) + 2 * M_PI, M_PI));
    bodyF_orientation_.Yaw(std::fmod((previous_bodyF_orientation_.Yaw() + rpyEulerRates(2) * Ts_) + 2 * M_PI, 2 * M_PI));
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
    ctb::LatLong2LocalNED(vehiclePos, altitude_, centroidLocation, worldF_com);

    //move the gps from COM to the antenna
    worldF_antenna = worldF_com + worldF_R_bodyF_ * config_->bodyF_gps_sensor_position;

    //add noise and come back to map coordinates
    worldF_antenna.x() += gpsNoiseX(generator);
    worldF_antenna.y() += gpsNoiseY(generator);
    worldF_antenna.z() += gpsNoiseZ(generator);

    ctb::LatLong gpsLatlong;
    double gpsAltitude;
    ctb::LocalNED2LatLong(worldF_antenna, centroidLocation, gpsLatlong, gpsAltitude);

    /*gpsMsg_.time = static_cast<double>(now_nanosecs / 1E9);
    gpsMsg_.track = vehicleTrack_;
    gpsMsg_.speed = vehicleSpeed_;
    gpsMsg_.latitude = gpsLatlong.latitude;
    gpsMsg_.longitude = gpsLatlong.longitude;
    gpsMsg_.altitude = gpsAltitude;
    gpsMsg_.gpsfixmode = ulisse_msgs::msg::GPSData::MODE3D; //3u
    */

    /////   COMPASS   /////
    std::normal_distribution<double> compassNoiseR(0.0, config_->sensorsNoise.compass_stdd.x());
    std::normal_distribution<double> compassNoiseP(0.0, config_->sensorsNoise.compass_stdd.y());
    std::normal_distribution<double> compassNoiseY(0.0, config_->sensorsNoise.compass_stdd.z());

    /*compassMsg_.stamp.sec = now_stamp_secs;
    compassMsg_.stamp.nanosec = now_stamp_nanosecs;
    compassMsg_.orientation.roll = bodyF_orientation_.Roll() + compassNoiseR(generator);
    compassMsg_.orientation.pitch = bodyF_orientation_.Pitch() + compassNoiseP(generator);
    compassMsg_.orientation.yaw = bodyF_orientation_.Yaw() + compassNoiseY(generator);
    */

    /////   MAGNETOMETER   /////
    Eigen::Vector3d m = { 23186.6 * 1E-9, 0.0 * 1E-9, 41122.0 * 1E-9 };  // Example of magnetic field at lat long: 44.4056° N, 8.9463° E

    Eigen::Vector3d ned_m = worldF_R_bodyF_.transpose() * m;

    std::normal_distribution<double> magnetometerNoiseX(0.0, config_->sensorsNoise.magnetometer_stdd.x());
    std::normal_distribution<double> magnetometerNoiseY(0.0, config_->sensorsNoise.magnetometer_stdd.y());
    std::normal_distribution<double> magnetometerNoiseZ(0.0, config_->sensorsNoise.magnetometer_stdd.z());

    /*magnetometerMsg_.stamp.sec = now_stamp_secs;
    magnetometerMsg_.stamp.nanosec = now_stamp_nanosecs;
    magnetometerMsg_.orthogonalstrength[0] = ned_m.x() + magnetometerNoiseX(generator);
    magnetometerMsg_.orthogonalstrength[1] = ned_m.y() + magnetometerNoiseY(generator);
    magnetometerMsg_.orthogonalstrength[2] = ned_m.z() + magnetometerNoiseZ(generator);
    */

    /////   IMU   /////
    /// Imu: Orientation
    //std::normal_distribution<double> orientusNoiseX(0.0, config_->sensorsNoise.orientus_stdd.x());
    //std::normal_distribution<double> orientusNoiseY(0.0, config_->sensorsNoise.orientus_stdd.y());
    //std::normal_distribution<double> orientusNoiseZ(0.0, config_->sensorsNoise.orientus_stdd.z());

    /********imuMsg_.stamp.sec = now_stamp_secs;
    imuMsg_.stamp.nanosec = now_stamp_nanosecs;
    Eigen::RotationMatrix bodyF_R_orientus = rml::EulerRPY(config_->bodyF_imu_sensor_pose.AngularVector()).ToRotationMatrix();
    rml::EulerRPY imuF_orientation = Eigen::RotationMatrix(bodyF_orientation_.ToRotationMatrix() * bodyF_R_orientus).ToEulerRPY();
    imuMsg_.orientation.roll = imuF_orientation.Roll() + compassNoiseR(generator);
    imuMsg_.orientation.pitch = imuF_orientation.Pitch() + compassNoiseP(generator);
    imuMsg_.orientation.yaw = imuF_orientation.Yaw() + compassNoiseY(generator);
    */

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

    //imuMsg_.stamp.sec = now_stamp_secs;
    //imuMsg_.stamp.nanosec = now_stamp_nanosecs;
    Eigen::Vector6d bodyF_relativeAcceleration;
    Eigen::Vector3d worldF_gravity = { 0.0, 0.0, -9.81 };
    bodyF_relativeAcceleration.segment(0, 3) = bodyF_relativeAcceleration_projected_.segment(0, 3) + worldF_R_bodyF_.transpose() * worldF_gravity;

    // !!!!Matrice di corpo rigido per le accelerazioni? (TODO -> RICERCA/RICAVA FORMULA)
    // !Eigen::Matrix6d bodyF_RBM_imu = config_->bodyF_imu_sensor_pose.LinearVector().GetRigidBodyMatrix();
    // !Eigen::Vector6d bodyF_relativeAcceleration_imu = bodyF_RBM_imu * bodyF_relativeAcceleration;
    Eigen::RotationMatrix bodyF_R_imu = rml::EulerRPY(config_->bodyF_imu_sensor_pose.AngularVector()).ToRotationMatrix();
    Eigen::Vector3d imuF_relativeLinearAcceleration = bodyF_R_imu.transpose() * bodyF_relativeAcceleration.LinearVector();

    /*imuMsg_.accelerometer[0] = imuF_relativeLinearAcceleration.x() + accelerometerNoiseX(generator);
    imuMsg_.accelerometer[1] = imuF_relativeLinearAcceleration.y() + accelerometerNoiseY(generator);
    imuMsg_.accelerometer[2] = imuF_relativeLinearAcceleration.z() + accelerometerNoiseZ(generator);
    */

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

    //imuMsg_.gyro[0] = bodyF_relativeAngularVelocity(0) + gyroNoiseX(generator) + bx;
    //imuMsg_.gyro[1] = bodyF_relativeAngularVelocity(1) + gyroNoiseY(generator) + by;
    //imuMsg_.gyro[2] = bodyF_relativeAngularVelocity(2) + gyroNoiseZ(generator) + bz;

    /// Imu: Magnetometer
    ///
    ///

    /////   DVL   /////
    std::normal_distribution<double> dvlNoiseX(0.0, config_->sensorsNoise.dvl_stdd.x());
    std::normal_distribution<double> dvlNoiseY(0.0, config_->sensorsNoise.dvl_stdd.y());
    std::normal_distribution<double> dvlNoiseZ(0.0, config_->sensorsNoise.dvl_stdd.z());

    //dvlMsg_.stamp.sec = now_stamp_secs;
    //dvlMsg_.stamp.nanosec = now_stamp_nanosecs;

    // Evaluating the rigid body matrix of the DVL w.r.t. the bodyFrame
    Eigen::Matrix6d bodyF_RBM_dvl = rml::RigidBodyMatrix(config_->bodyF_dvl_sensor_pose.LinearVector());
    // Evaluating the components of the velocity, in the DVL frame
    Eigen::Vector6d bodyF_relativeVelocity_dvl = bodyF_RBM_dvl * bodyF_relativeVelocity_;
    // Evaluating the rotation matrix of the DVL w.r.t to the bodyFrame
    Eigen::RotationMatrix bodyF_R_dvl = rml::EulerRPY(config_->bodyF_dvl_sensor_pose.AngularVector()).ToRotationMatrix();
    // Rotating the components according to the sensor positioning
    Eigen::Vector3d dvlF_relativeLinearVelocity = bodyF_R_dvl.transpose() * bodyF_relativeVelocity_dvl.LinearVector();

    //dvlMsg_.water_tracking[0] = dvlF_relativeLinearVelocity(0) + dvlNoiseX(generator);
   // dvlMsg_.water_tracking[1] = dvlF_relativeLinearVelocity(1) + dvlNoiseY(generator);
    //dvlMsg_.water_tracking[2] = dvlF_relativeLinearVelocity(2) + dvlNoiseZ(generator);

    // Projecting back the absolute vehicle velocity in worldFrame onto the bodyFrame
    Eigen::Vector6d bodyF_absoluteVelocity_ = bodyF_orientation_.ToRotationMatrix().CartesianRotationMatrix().transpose() * worldF_velocity_;
    // Evaluating the components of the velocity, in the DVL frame
    Eigen::Vector6d bodyF_absoluteVelocity_dvl = bodyF_RBM_dvl * bodyF_absoluteVelocity_;
    // Rotating the components according to the sensor positioning
    Eigen::Vector3d dvlF_absoluteLinearVelocity = bodyF_R_dvl.transpose() * bodyF_absoluteVelocity_dvl.LinearVector();

    //dvlMsg_.bottom_velocity[0] = dvlF_absoluteLinearVelocity(0) + dvlNoiseX(generator);
    //dvlMsg_.bottom_velocity[1] = dvlF_absoluteLinearVelocity(1) + dvlNoiseY(generator);
    //dvlMsg_.bottom_velocity[2] = dvlF_absoluteLinearVelocity(2) + dvlNoiseZ(generator);


    /////   FOG   /////
    std::normal_distribution<double> fogNoise(0.0, config_->sensorsNoise.fog_stdd);

    /*fogMsg_.stamp.sec = now_stamp_secs;
    fogMsg_.stamp.nanosec = now_stamp_nanosecs;
    fogMsg_.angular_velocity = bodyF_relativeAngularVelocity(2) + fogNoise(generator);


    /////   AMBIENT   /////
    ambsensMsg_.stamp.sec = now_stamp_secs;
    ambsensMsg_.stamp.nanosec = now_stamp_nanosecs;
    ambsensMsg_.temperaturectrlbox = 23.0 + (rand() / static_cast<double>(RAND_MAX)) * 2.0;
    ambsensMsg_.humidityctrlbox = 50.0 + (rand() / static_cast<double>(RAND_MAX)) * 2.0;
    */


    // Fill the ground truth msg
    groundTruthMsg_.stamp.sec = now_stamp_secs;
    groundTruthMsg_.stamp.nanosec = now_stamp_nanosecs;
    groundTruthMsg_.inertialframe_linear_position.latlong.latitude = vehiclePos.latitude;
    groundTruthMsg_.inertialframe_linear_position.latlong.longitude = vehiclePos.longitude;
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
    groundTruthMsg_.inertialframe_water_current[1] = worldF_waterVelocity_[1];
    groundTruthMsg_.gyro_bias[0] = bx;
    groundTruthMsg_.gyro_bias[1] = by;
    groundTruthMsg_.gyro_bias[2] = bz;
    groundTruthMsg_.n_p = n_p_;
    groundTruthMsg_.n_s = n_s_;

    //motor ref
    /*appliedMotorRefMsg_.left_percentage = hp_;
    appliedMotorRefMsg_.right_percentage = hs_;
    
    motorsDataMsg_.stamp.sec = now_stamp_secs;
    motorsDataMsg_.stamp.nanosec = now_stamp_nanosecs;
    motorsDataMsg_.left.motor_speed = n_p_; //ulisseModel.PercentageToRPM(hp_);
    motorsDataMsg_.right.motor_speed = n_s_; //ulisseModel.PercentageToRPM(hs_);
    */
}

void VehicleSimulator::PublishSensors()
{
    //microLoopCountPub_->publish(microLoopCountMsg_);
    simulatedSystemPub_->publish(groundTruthMsg_);
    //appliedMotorRefPub_->publish(appliedMotorRefMsg_);
    //motorsDataPub_->publish(motorsDataMsg_);

    /*if (static_cast<int>(timestamp_count_ / 20) > gpsPubCounter_) {
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

    if (static_cast<int>(timestamp_count_ / 20) > ambientPubCounter_) {
        ambientPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        ambsensPub_->publish(ambsensMsg_);
    }

    if (static_cast<int>(timestamp_count_ / 20) > magnetometerPubCounter_) {
        magnetometerPubCounter_ = static_cast<int>(timestamp_count_ / 20);
        magnetometerPub_->publish(magnetometerMsg_);
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

/* void VehicleSimulator::ThrustersReferenceCB(const rov_msgs::msg::ThrustersReference::SharedPtr msg)
{
    hp_ = msg->left_percentage;
    hs_ = msg->right_percentage;

    motorTimeout_.Start();
}
*/
}
