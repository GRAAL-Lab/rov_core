#include "rov_ctrl/dynamic_rov_controller.hpp"

#include <jsoncpp/json/json.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
    filterSub_ = this->create_subscription<rov_msgs::msg::NavFilterData>(rov_msgs::topicnames::nav_filter_data, 10,
        std::bind(&DynamicRovController::FilterDataCB, this, _1));
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

    //Service
    srvUserInput_ = this->create_service<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service, std::bind(&DynamicRovController::CommandsHandler, this, _1, _2, _3));

    dcl_conf = std::make_shared<DCLConfiguration>();

    //ROV params configuration
    if (!LoadDclConfiguration(dcl_conf, confFileName_)) {
        std::cerr << "Failed to load DCL Configuration. Check the parameters in the conf file" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << tc::brown << *dcl_conf << tc::none << std::endl;

    rovModel_.params = dcl_conf->rovModel;

    std::cout << "Qparam =" << rovModel_.params.Q << std::endl;
    std::cout << "Tparam =" << rovModel_.params.T << std::endl;
    std::cout << "Kparam =" << rovModel_.params.K<< std::endl;

    if(rovModel_.params.heavyConf){
        rov_allocationMatrix.resize(6,8);
        thruster_voltage_.resize(8,1);
        volts_.resize(8,1);
    }
    else {
        rov_allocationMatrix.resize(6,6);
        thruster_voltage_.resize(6,1);
        volts_.resize(6,1);
    }
    //rov_allocationMatrix = T*K*Q;
    rov_allocationMatrix = rovModel_.params.T * rovModel_.params.K * rovModel_.params.Q;

    //Controller inizialization
    //ClassicPidControlInizialization(dcl_conf, sampleTime_, pidSurgeCP, pidYawRateCP);

    if (dcl_conf->ctrlMode == ControlMode::ThrusterMapping) {
        //ThrusterMappingInizialization(dcl_conf, sampleTime_, pidSurgeTM);
    } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
        ClassicPidControlInizialization(dcl_conf, sampleTime_, pidSurgeCP_,pidSwayCP_, pidHeaveCP_,
                                        pidRollRateCP_, pidPitchRateCP_, pidYawRateCP_);
    } else {
        //ComputedTorqueControlInizialization(dcl_conf, sampleTime_, pidSurgeCT, pidYawRateCT);
    }

    //srvResetConf_ = this->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_dcl_conf_service,
    //    std::bind(&DynamicVehicleController::ResetConfHandler, this, _1, _2, _3));


    // Main function timer   
    int msRunPeriod = 1.0/(dcl_conf->controlLoopRate) * 1000;
    std::cout << "Controller Rate: " << dcl_conf->controlLoopRate << "Hz" << std::endl;
    runTimer_ = this->create_wall_timer(std::chrono::milliseconds(msRunPeriod), std::bind(&DynamicRovController::Run, this));

}

DynamicRovController::~DynamicRovController() {

}


void DynamicRovController::Run()
{

    // Evaluating the total surge, taking into account the water current
    Eigen::RotationMatrix Rz, Ry, Rx;
    Rz << cos(filterData.bodyframe_angular_position.yaw), -sin(filterData.bodyframe_angular_position.yaw), 0,
        sin(filterData.bodyframe_angular_position.yaw), cos(filterData.bodyframe_angular_position.yaw), 0,
        0, 0, 1;

    Ry << cos(filterData.bodyframe_angular_position.pitch), 0, sin(filterData.bodyframe_angular_position.pitch),
        0, 1, 0,
        -sin(filterData.bodyframe_angular_position.pitch), 0, cos(filterData.bodyframe_angular_position.pitch);

    Rx << 1, 0, 0,
        0, cos(filterData.bodyframe_angular_position.roll), -sin(filterData.bodyframe_angular_position.roll),
        0, sin(filterData.bodyframe_angular_position.roll), cos(filterData.bodyframe_angular_position.roll);

    Eigen::RotationMatrix worldF_R_bodyF_ = Rz * Ry * Rx;
    Eigen::RotationMatrix bodyF_R_worldF = worldF_R_bodyF_.transpose();

    Eigen::Vector3d water_current_b = bodyF_R_worldF * Eigen::Vector3d(filterData.inertialframe_water_current.data());


    double absSurgeFbk = filterData.bodyframe_linear_velocity[0] + water_current_b[0];   // ?! è la velocità relativa all'acqua?
    double relSurgeFbk = filterData.bodyframe_linear_velocity[0];
    double absSwayFbk = filterData.bodyframe_linear_velocity[1] + water_current_b[1];   // ?! è la velocità relativa all'acqua?
    double relSwayFbk = filterData.bodyframe_linear_velocity[1];
    double absHeaveFbk = filterData.bodyframe_linear_velocity[2] + water_current_b[2];   // ?! è la velocità relativa all'acqua?
    double relHeaveFbk = filterData.bodyframe_linear_velocity[2];

    double rollRateFbk = filterData.bodyframe_angular_velocity[0];
    double pitchRateFbk = filterData.bodyframe_angular_velocity[1];
    double yawRateFbk = filterData.bodyframe_angular_velocity[2];

    if (vehicleStatus.vehicle_state != rov::states::ID::halt) {
        //ThrusterMapping mode
        if (dcl_conf->ctrlMode == ControlMode::Forces) {
            //std::cout << "ControlMode hold" << std::endl;
            if(vehicleStatus.vehicle_state == rov::states::ID::hold){
                Eigen::Vector6d tau; tau.setZero();
                //std::cout << "MoveByForce before" << std::endl;
                MoveByForce(tau,thruster_voltage_);
                //std::cout << "MoveByForce after" << std::endl;
                motion_direction = rov::inputs::ID::hold;
                //std::cout << "ControlMode hold" << std::endl;
            }
            else if (vehicleStatus.vehicle_state == rov::states::ID::velocity){
                Eigen::Vector6d tau; tau.setZero();

                switch(motion_direction){
                case rov::inputs::ID::halt :{
                    //vehicleStatus.vehicle_state = rov::states::ID::halt;
                } break;
                case rov::inputs::ID::hold :{
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::forward :{
                    tau[0] = 10.0;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::backward :{
                    tau[0] = -10.0;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::up :{
                    tau[2] = -10.0;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::down :{
                    tau[2] = 10.0;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::left :{
                    tau[1] = -10.0;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::right :{
                    tau[1] = 10.0;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::turn_left :{
                    tau[5] = -0.1;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                case rov::inputs::ID::turn_right :{
                    tau[5] = 0.1;
                    MoveByForce(tau,thruster_voltage_);
                } break;
                default:{
                    std::cout << "direction not supported (user input)" << std::endl;
                } break;
                }
            }


        } else if (dcl_conf->ctrlMode == ControlMode::ClassicPIDControl) {
            Eigen::Vector6d dirV;
            dirV.setZero();
            if(vehicleStatus.vehicle_state == rov::states::ID::hold){
                Eigen::Vector6d tau; tau.setZero();
                MoveByForce(tau,thruster_voltage_);
                motion_direction = rov::inputs::ID::hold;
            }
            else if (vehicleStatus.vehicle_state == rov::states::ID::velocity){

                SetDirectionVector(dirV);
                //Dynamic Pids

                // different tau for normal configuration
                // tau for heavy configuration

                tau << dirV[0] * pidSurgeCP_.Compute(referenceVelocities.desired_surge, absSurgeFbk),
                    dirV[1] * pidSwayCP_.Compute(referenceVelocities.desired_sway, absSwayFbk),
                    dirV[2] * pidHeaveCP_.Compute(referenceVelocities.desired_heave, absHeaveFbk),
                    dirV[3] * pidRollRateCP_.Compute(referenceVelocities.desired_roll_rate, rollRateFbk),
                    dirV[4] * pidPitchRateCP_.Compute(referenceVelocities.desired_pitch_rate, pitchRateFbk),
                    dirV[5] * pidYawRateCP_.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk);

                thruster_voltage_ = rovModel_.ThusterAllocation(tau);
            }
            else{}

            Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();

            feedbackVel(0) = absSurgeFbk;
            feedbackVel(1) = absSwayFbk;
            feedbackVel(2) = absHeaveFbk;
            feedbackVel(3) = rollRateFbk;
            feedbackVel(4) = pitchRateFbk;
            feedbackVel(5) = yawRateFbk;

            rovModel_.ThrustersSaturation(thruster_voltage_, 1.0);

            //Fill the classic dynamic pid contol msg
            auto t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            classicPidControlMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            classicPidControlMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
            classicPidControlMsg.desired_surge = dirV[0]*referenceVelocities.desired_surge;
            classicPidControlMsg.desired_sway = dirV[1]*referenceVelocities.desired_sway;
            classicPidControlMsg.desired_heave = dirV[2]*referenceVelocities.desired_heave;
            classicPidControlMsg.feedback_surge = absSurgeFbk;
            classicPidControlMsg.feedback_sway = absSwayFbk;
            classicPidControlMsg.feedback_heave = absHeaveFbk;
            classicPidControlMsg.out_pid_surge = pidSurgeCP_.GetOutput();
            classicPidControlMsg.out_pid_sway = pidSwayCP_.GetOutput();
            classicPidControlMsg.out_pid_heave = pidHeaveCP_.GetOutput();
            classicPidControlMsg.desired_roll_rate = dirV[3]*referenceVelocities.desired_roll_rate;
            classicPidControlMsg.desired_pitch_rate = dirV[4]*referenceVelocities.desired_yaw_rate;
            classicPidControlMsg.desired_yaw_rate = dirV[5]*referenceVelocities.desired_yaw_rate;
            classicPidControlMsg.feedback_roll_rate = rollRateFbk;
            classicPidControlMsg.feedback_pitch_rate = pitchRateFbk;
            classicPidControlMsg.feedback_yaw_rate = yawRateFbk;
            classicPidControlMsg.out_pid_roll_rate = pidRollRateCP_.GetOutput();
            classicPidControlMsg.out_pid_pitch_rate = pidPitchRateCP_.GetOutput();
            classicPidControlMsg.out_pid_yaw_rate = pidYawRateCP_.GetOutput();

            classicPidControlMsg.tau = { tau[0], tau[1], tau[2], tau[3],tau[4], tau[5]};
            classicPidControlMsg.motor_percentage.first_percentage = thruster_voltage_[0];
            classicPidControlMsg.motor_percentage.second_percentage = thruster_voltage_[1];
            classicPidControlMsg.motor_percentage.third_percentage = thruster_voltage_[2];
            classicPidControlMsg.motor_percentage.forth_percentage = thruster_voltage_[3];
            classicPidControlMsg.motor_percentage.fifth_percentage = thruster_voltage_[4];
            classicPidControlMsg.motor_percentage.sixth_percentage = thruster_voltage_[5];
            if(rovModel_.params.heavyConf){
                classicPidControlMsg.motor_percentage.seventh_percentage = thruster_voltage_[6];
                classicPidControlMsg.motor_percentage.eighth_percentage = thruster_voltage_[7];
            }

            classicPidControlPub_->publish(classicPidControlMsg);

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
        else{}

        rovModel_.ThrustersSaturation(thruster_voltage_, 1.0);
        //std::cout << "thrusterVoltage " << thruster_voltage_ <<std::endl;
        thrustersReference.first_percentage = thruster_voltage_[0];
        thrustersReference.second_percentage = thruster_voltage_[1];
        thrustersReference.third_percentage = thruster_voltage_[2];
        thrustersReference.forth_percentage = thruster_voltage_[3];
        thrustersReference.fifth_percentage = thruster_voltage_[4];
        thrustersReference.sixth_percentage = thruster_voltage_[5];
        if(rovModel_.params.heavyConf){
            thrustersReference.seventh_percentage = thruster_voltage_[6];
            thrustersReference.eighth_percentage = thruster_voltage_[7];
        }
    } else {
        //std::cout << "HaltMode " << std::endl;
        thrustersReference.first_percentage = 0.0;
        thrustersReference.second_percentage = 0.0;
        thrustersReference.third_percentage = 0.0;
        thrustersReference.forth_percentage = 0.0;
        thrustersReference.fifth_percentage = 0.0;
        thrustersReference.sixth_percentage = 0.0;
        if(rovModel_.params.heavyConf){
            thrustersReference.seventh_percentage = 0.0;
            thrustersReference.eighth_percentage = 0.0;
        }

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

void DynamicRovController::CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,const std::shared_ptr<rov_msgs::srv::UserInput::Request> request,
                                           std::shared_ptr<rov_msgs::srv::UserInput::Response> response){
    // Create a callback function for when service requests are received.
    (void) request_header;
    RCLCPP_INFO(this->get_logger(), "Incoming request: %d", request->motion_type);
    std::stringstream logg;
    logg << "Incoming request: " << request->motion_type;

    motion_direction = request->motion_type;

    response->res = "command received";
    RCLCPP_INFO(this->get_logger(), "Service Response: %s", response->res.c_str());
}

void DynamicRovController::ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime,
                                                           ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidSway, ctb::DigitalPID& pidHeave,
                                                           ctb::DigitalPID& pidRollRate, ctb::DigitalPID& pidPitchRate, ctb::DigitalPID& pidYawRate)
{
    pidSurge.Initialize(conf->classicPidControl.pidGainsSurge, sampleTime, conf->classicPidControl.pidSatSurge);
    pidSway.Initialize(conf->classicPidControl.pidGainsSway, sampleTime, conf->classicPidControl.pidSatSway);
    pidHeave.Initialize(conf->classicPidControl.pidGainsHeave, sampleTime, conf->classicPidControl.pidSatHeave);
    pidRollRate.Initialize(conf->classicPidControl.pidGainsRollRate, sampleTime, conf->classicPidControl.pidSatRollRate);
    pidPitchRate.Initialize(conf->classicPidControl.pidGainsPitchRate, sampleTime, conf->classicPidControl.pidSatPitchRate);
    pidYawRate.Initialize(conf->classicPidControl.pidGainsYawRate, sampleTime, conf->classicPidControl.pidSatYawRate);
}

void DynamicRovController::MoveByForce(const Eigen::Vector6d &force, Eigen::VectorXd &volt){
    Eigen::Vector6d tau;
    tau[0] = vehicleForces.bodyframe_g[0];
    tau[1] = vehicleForces.bodyframe_g[1];
    tau[2] = vehicleForces.bodyframe_g[2];
    tau[3] = vehicleForces.bodyframe_g[3];
    tau[4] = vehicleForces.bodyframe_g[4];
    tau[5] = vehicleForces.bodyframe_g[5];

    volt = rovModel_.ThusterAllocation(- tau + force);

    //Eigen::JacobiSVD<Eigen::MatrixXd> svd( rov_allocationMatrix, Eigen::ComputeFullV | Eigen::ComputeFullU );
    //std::cout << "rov_allocationMatrix = " << rov_allocationMatrix << std::endl;
    //volt = svd.solve(- tau + force);
    //std::cout << "volt = " << volt << std::endl;
    //std::cout << "T*K*volt = " << rov_allocationMatrix*volt << std::endl;

}

void DynamicRovController::SetDirectionVector(Eigen::Vector6d &d_vect){
    d_vect.setZero();
    switch(motion_direction){
    case rov::inputs::ID::halt :{
        //vehicleStatus.vehicle_state = rov::states::ID::halt;
    } break;
    case rov::inputs::ID::hold :{
        d_vect.setZero();
    } break;
    case rov::inputs::ID::forward :{
        d_vect[0] = 1;
    } break;
    case rov::inputs::ID::backward :{
        d_vect[0] = -1;
    } break;
    case rov::inputs::ID::up :{
        directionVector_[2] = 1;
    } break;
    case rov::inputs::ID::down :{
        d_vect[2] = -1;
    } break;
    case rov::inputs::ID::left :{
        d_vect[1] = -1;
    } break;
    case rov::inputs::ID::right :{
        d_vect[1] = 1;
    } break;
    case rov::inputs::ID::turn_left :{
        d_vect[5] = -1;
    } break;
    case rov::inputs::ID::turn_right :{
        d_vect[5] = 1;
    } break;
    default:{
        std::cout << "direction not supported (user input)" << std::endl;
    } break;
    }
}

void DynamicRovController::VehicleStatusCB(const rov_msgs::msg::VehicleStatus::SharedPtr msg) { vehicleStatus = *msg; }

void DynamicRovController::ReferenceVelocitiesCB(const rov_msgs::msg::ReferenceVelocities::SharedPtr msg) { referenceVelocities = *msg; }

void DynamicRovController::VehicleForcesCB(const rov_msgs::msg::Forces::SharedPtr msg) { vehicleForces = *msg; }

void DynamicRovController::FilterDataCB(const rov_msgs::msg::NavFilterData::SharedPtr msg) { filterData = *msg; }

} // namespace rov
