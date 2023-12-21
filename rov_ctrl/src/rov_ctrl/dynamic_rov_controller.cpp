#include "rov_ctrl/dynamic_rov_controller.hpp"

#include <jsoncpp/json/json.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

//#include "ulisse_ctrl/ulisse_defines.hpp"
//#include "ulisse_ctrl/configuration.hpp"

//#include "ulisse_msgs/terminal_utils.hpp"
#include "rov_msgs/topicnames.hpp"
#include "rov_msgs/terminal_utils.hpp"
#include "rov_ctrl/rov_defines.hpp" // appear topicnames!!


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rov {

DynamicRovController::DynamicRovController(std::string file_name)
    : Node("dynamic_control_node")
{


    confFileName_ = file_name;

    //Subscribers
    //filterSub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10,
    //    std::bind(&DynamicVehicleController::FilterDataCB, this, _1));
    //vehicleStatusSub_ = this->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10,
    //    std::bind(&DynamicVehicleController::VehicleStatusCB, this, _1));
    referenceVelocitiesSub_ = this->create_subscription<rov_msgs::msg::ReferenceVelocities>(rov_msgs::topicnames::reference_velocities, 10,
        std::bind(&DynamicRovController::ReferenceVelocitiesCB, this, _1));
    vehicleStatusSub_ = this->create_subscription<rov_msgs::msg::VehicleStatus>(rov_msgs::topicnames::vehicle_status, 10,
                                                                                   std::bind(&DynamicRovController::VehicleStatusCB, this, _1));
    vehicleForcesSub_ = this->create_subscription<rov_msgs::msg::Forces>(rov_msgs::topicnames::forces, 10,
                                                                                std::bind(&DynamicRovController::VehicleForcesCB, this, _1));
    //Publishers
    thrusterDataPub_ = this->create_publisher<rov_msgs::msg::ThrustersReference>(rov_msgs::topicnames::llc_thrusters_reference_perc, 1);
    //thrusterMappigPub_ = this->create_publisher<ulisse_msgs::msg::ThrusterMappingControl>(ulisse_msgs::topicnames::thruster_mapping_control, 1);
    //simulatedVelocitySensorPub_ = this->create_publisher<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 1);
    classicPidControlPub_ = this->create_publisher<rov_msgs::msg::DynamicPidControl>(rov_msgs::topicnames::classic_pid_control, 1);
    //computedTorqueControlPub_ = this->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::computed_torque_control, 1);


    dcl_conf = std::make_shared<DCLConfiguration>();

    //ROV params configuration
    if (!LoadDclConfiguration(dcl_conf, confFileName_)) {
        std::cerr << "Failed to load DCL Configuration. Check the parameters in the conf file" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << tc::brown << *dcl_conf << tc::none << std::endl;

    rovModel_.params = dcl_conf->rovModel;
    Eigen::Matrix6d T,K;
    T.row(0) = rovModel_.params.T_vector.segment(0,6);
    T.row(1) = rovModel_.params.T_vector.segment(6,6);
    T.row(2) = rovModel_.params.T_vector.segment(12,6);
    T.row(3) = rovModel_.params.T_vector.segment(18,6);
    T.row(4) = rovModel_.params.T_vector.segment(24,6);
    T.row(5) = rovModel_.params.T_vector.segment(30,6);
    K = rovModel_.params.K_diag.asDiagonal();

    rov_allocationMatrix = T*K;
    //Controller inizialization
    ClassicPidControlInizialization(dcl_conf, sampleTime_, pidSurgeCP, pidYawRateCP);


    if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {
        //ThrusterMappingInizialization(dcl_conf, sampleTime_, pidSurgeTM);
    } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
        ClassicPidControlInizialization(dcl_conf, sampleTime_, pidSurgeCP, pidYawRateCP);
    } else {
        //ComputedTorqueControlInizialization(dcl_conf, sampleTime_, pidSurgeCT, pidYawRateCT);
    }

    //srvResetConf_ = this->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_dcl_conf_service,
    //    std::bind(&DynamicVehicleController::ResetConfHandler, this, _1, _2, _3));


    // Main function timer   
    int msRunPeriod = 1.0/(dcl_conf->controlLoopRate) * 1000;
    //std::cout << "Controller Rate: " << rate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&DynamicRovController::Run, this));

}

DynamicRovController::~DynamicRovController() {

}


void DynamicRovController::Run()
{

    // Evaluating the total surge, taking into account the water current
    //Eigen::Rotation2D<double> wRb = Eigen::Rotation2D<double>(filterData.bodyframe_angular_position.yaw);
    //Eigen::Vector2d water_current_b = wRb.inverse() * Eigen::Vector2d(filterData.inertialframe_water_current.data());

    /*
    //double absSurgeFbk = filterData.bodyframe_linear_velocity[0] + water_current_b[0];   // ?! è la velocità relativa all'acqua?
    //double relSurgeFbk = filterData.bodyframe_linear_velocity[0];
    //double yawRateFbk = filterData.bodyframe_angular_velocity[2];
    */
    //double absSurgeFbk;// = filterData.bodyframe_linear_velocity[0] + water_current_b[0];   // ?! è la velocità relativa all'acqua?
    //double relSurgeFbk;
    //double yawRateFbk;

    Eigen::Vector6d thruster_voltage;

    if (vehicleStatus.vehicle_state != rov::states::ID::halt) {
        //ThrusterMapping mode
        if (dcl_conf->ctrlMode == ControlMode::Forces) {
            //std::cout << "ControlMode hold" << std::endl;
            if(vehicleStatus.vehicle_state == rov::states::ID::hold){
                Eigen::Vector6d tau; tau.setZero();
                MoveByForce(tau,thruster_voltage);
                //std::cout << "ControlMode hold" << std::endl;
            }
            else if(vehicleStatus.vehicle_state == rov::states::ID::forward){
                Eigen::Vector6d tau; tau.setZero();
                tau[0] = 10.0;
                MoveByForce(tau,thruster_voltage);
            }
            else if(vehicleStatus.vehicle_state == rov::states::ID::backward){
                Eigen::Vector6d tau; tau.setZero();
                tau[0] = -10.0;
                MoveByForce(tau,thruster_voltage);
            }
            else if(vehicleStatus.vehicle_state == rov::states::ID::up){
                Eigen::Vector6d tau; tau.setZero();
                tau[2] = -10.0;
                MoveByForce(tau,thruster_voltage);
            }
            else if(vehicleStatus.vehicle_state == rov::states::ID::down){
                Eigen::Vector6d tau; tau.setZero();
                tau[2] = 10.0;
                MoveByForce(tau,thruster_voltage);
            }
            else if(vehicleStatus.vehicle_state == rov::states::ID::left){
                Eigen::Vector6d tau; tau.setZero();
                tau[5] = -0.1;
                MoveByForce(tau,thruster_voltage);
            }
            else if(vehicleStatus.vehicle_state == rov::states::ID::right){
                Eigen::Vector6d tau; tau.setZero();
                tau[5] = 0.1;
                MoveByForce(tau,thruster_voltage);
            }

        } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
            //Dynamic Pids
            /*
            Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();

            tau = { pidSurgeCP.Compute(referenceVelocities.desired_surge, absSurgeFbk), pidYawRateCP.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };

            feedbackVel(0) = absSurgeFbk;
            feedbackVel(5) = yawRateFbk;
            double outleft, outright;

            Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
            ulisseModel.InverseMotorsEquations(feedbackVel, forces, outleft, outright);
            ulisseModel.ThrustersSaturation(outleft, outright, -dcl_conf->thrusterPercLimit, dcl_conf->thrusterPercLimit, thrustersReference.left_percentage, thrustersReference.right_percentage);

            //Fill the classic dynamic pid contol msg
            auto t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            classicPidControlMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            classicPidControlMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
            classicPidControlMsg.desired_surge = referenceVelocities.desired_surge;
            classicPidControlMsg.feedback_surge = absSurgeFbk;
            classicPidControlMsg.out_pid_surge = pidSurgeCP.GetOutput();
            classicPidControlMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
            classicPidControlMsg.feedback_yaw_rate = yawRateFbk;
            classicPidControlMsg.out_pid_yaw_rate = pidYawRateCP.GetOutput();
            classicPidControlMsg.forces = { forces[0], forces[1] };
            classicPidControlMsg.tau = { tau[0], tau[1] };
            classicPidControlMsg.motor_percentage.left_percentage = outleft;
            classicPidControlMsg.motor_percentage.right_percentage = outright;

            classicPidControlPub_->publish(classicPidControlMsg);

            //fill the feedback for the nav filter    <----------- CHECK TODO
            //simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;
            //simulatedVelocitySensorPub_->publish(simulatedVelocitySensor);
            */

        } /*else if (dcl_conf->ctrlMode == ControlMode::ComputedTorque) {


            tau = { pidSurgeCT.Compute(referenceVelocities.desired_surge, absSurgeFbk), pidYawRateCT.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };

            // using relative surge velocity for the feedforward term
            Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();
            feedbackVel(0) = relSurgeFbk;
            feedbackVel(5) = yawRateFbk;

            Eigen::Vector3d tauDrag = ulisseModel.ComputeCoriolisAndDragForces(feedbackVel);
            
            //std::cerr << "tau PID:  F = " << tau[0] << " | N = " << tau[1] << std::endl;
            //std::cerr << "tau CT :  F = " << tauDrag[0] << " | N = " << tauDrag[2] << std::endl;

            tau += Eigen::Vector2d(tauDrag[0], tauDrag[2]);
            double outLeft, outRight;

            Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
            ulisseModel.InverseMotorsEquations(feedbackVel, forces, outLeft, outRight);
            ulisseModel.ThrustersSaturation(outLeft, outRight, -dcl_conf->thrusterPercLimit, dcl_conf->thrusterPercLimit, thrustersReference.left_percentage, thrustersReference.right_percentage);


            //Fill the classic dynamic pid contol msg
            auto t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            computedTorqueMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            computedTorqueMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
            computedTorqueMsg.desired_surge = referenceVelocities.desired_surge;
            computedTorqueMsg.feedback_surge = absSurgeFbk;
            computedTorqueMsg.out_pid_surge = pidSurgeCP.GetOutput();
            computedTorqueMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
            computedTorqueMsg.feedback_yaw_rate = yawRateFbk;
            computedTorqueMsg.out_pid_yaw_rate = pidYawRateCP.GetOutput();
            computedTorqueMsg.forces = { forces[0], forces[1] };
            computedTorqueMsg.tau = { tau[0], tau[1] };
            computedTorqueMsg.motor_percentage.left_percentage = outLeft;
            computedTorqueMsg.motor_percentage.right_percentage = outRight;
            computedTorqueControlPub_->publish(computedTorqueMsg);

            //fill the feedback for the nav filter
            simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;
            simulatedVelocitySensorPub_->publish(simulatedVelocitySensor);
        }*/
        rovModel_.ThrustersSaturation(thruster_voltage, 1.0);
        std::cout << "thrusterVoltage " << thruster_voltage <<std::endl;
        thrustersReference.first_percentage = thruster_voltage[0];
        thrustersReference.second_percentage = thruster_voltage[1];
        thrustersReference.third_percentage = thruster_voltage[2];
        thrustersReference.forth_percentage = thruster_voltage[3];
        thrustersReference.fifth_percentage = thruster_voltage[4];
        thrustersReference.sixth_percentage = thruster_voltage[5];
    } else {
        //std::cout << "HaltMode " << std::endl;
        thrustersReference.first_percentage = 0.0;
        thrustersReference.second_percentage = 0.0;
        thrustersReference.third_percentage = 0.0;
        thrustersReference.forth_percentage = 0.0;
        thrustersReference.fifth_percentage = 0.0;
        thrustersReference.sixth_percentage = 0.0;

        /*if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {
            pidSurgeTM.Reset();
        } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
            pidSurgeCP.Reset();
            pidYawRateCP.Reset();
        } else {
            pidSurgeCT.Reset();
            pidYawRateCT.Reset();
        }*/
    }


    PublishControl();

}

void DynamicRovController::PublishControl()
{
    auto tNow = std::chrono::system_clock::now();
    long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
    auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
    auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
    thrustersReference.stamp.sec = now_stamp_secs;
    thrustersReference.stamp.nanosec = now_stamp_nanosecs;
    thrusterDataPub_->publish(thrustersReference);
}

bool DynamicRovController::LoadDclConfiguration(std::shared_ptr<DCLConfiguration> dcl_conf, std::string filename)
{
    libconfig::Config confObj;

    // Read the ROV_CTRL config file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rov_ctrl");
    std::string confPath = package_share_directory + "/conf/" + filename;
    std::cout << "PATH TO ROV_CTRL CONF FILE (DCL): " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return false;
    }
    if (!dcl_conf->LoadConfiguration(confObj))
        return false;

    // Read the ROV_MODEL config file
    package_share_directory = ament_index_cpp::get_package_share_directory("underwater_vehicle_model");
    confPath = package_share_directory + "/conf/blueROV.conf";
    std::cout << "PATH TO ROV_MODEL CONF FILE (DCL): " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return false;
    }

    if (!dcl_conf->ConfigureRovModel(confObj))
        return false;

    return true;
}
/*
void DynamicRovController::ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid)
{
    pid.Initialize(conf->thrusterMapping.pidGainsSurge, sampleTime, conf->thrusterMapping.pidSatSurge);
}
*/
void DynamicRovController::ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate)
{
    pidSurge.Initialize(conf->classicPidControl.pidGainsSurge, sampleTime, conf->classicPidControl.pidSatSurge);
    pidYawRate.Initialize(conf->classicPidControl.pidGainsYawRate, sampleTime, conf->classicPidControl.pidSatYawRate);
}

void DynamicRovController::MoveByForce(const Eigen::Vector6d &force, Eigen::Vector6d &volt){
    Eigen::Vector6d tau;
    tau[0] = vehicleForces.bodyframe_g[0];
    tau[1] = vehicleForces.bodyframe_g[1];
    tau[2] = vehicleForces.bodyframe_g[2];
    tau[3] = vehicleForces.bodyframe_g[3];
    tau[4] = vehicleForces.bodyframe_g[4];
    tau[5] = vehicleForces.bodyframe_g[5];

    Eigen::JacobiSVD<Eigen::MatrixXd> svd( rov_allocationMatrix, Eigen::ComputeFullV | Eigen::ComputeFullU );
    std::cout << "rov_allocationMatrix = " << rov_allocationMatrix << std::endl;
    volt = svd.solve(- tau + force);
    std::cout << "T*K*volt = " << rov_allocationMatrix*volt << std::endl;
}

void DynamicRovController::VehicleStatusCB(const rov_msgs::msg::VehicleStatus::SharedPtr msg) { vehicleStatus = *msg; }

void DynamicRovController::ReferenceVelocitiesCB(const rov_msgs::msg::ReferenceVelocities::SharedPtr msg) { referenceVelocities = *msg; }

void DynamicRovController::VehicleForcesCB(const rov_msgs::msg::Forces::SharedPtr msg) { vehicleForces = *msg; }

} // namespace rov
