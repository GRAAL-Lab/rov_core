#ifndef VISUALIZER_DEFINES_H
#define VISUALIZER_DEFINES_H

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <random>

#include "rov_model/rov_model.hpp"
#include "rov_msgs/futils.hpp"

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

namespace rov {

/*
struct SinusoidalWave {
    double A, C, f;

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {
        if (!ctb::GetParam(confObj, A, "A"))
            return false;
        if (!ctb::GetParam(confObj, C, "C"))
            return false;
        if (!ctb::GetParam(confObj, f, "f"))
            return false;

        return true;
    }
};

struct SensorsNoise {
    Eigen::Vector3d gps_stdd;
    Eigen::Vector3d compass_stdd;
    Eigen::Vector3d magnetometer_stdd;
    Eigen::Vector3d gyro_stdd;
    Eigen::Vector3d accelerometer_stdd;
    Eigen::Vector3d dvl_stdd;
    Eigen::Vector3d orientus_stdd;
    double fog_stdd;
    double pressure_stdd;

    SinusoidalWave bx, by, bz;

    bool ConfigureFromFile(const libconfig::Setting& confObj) noexcept(false)
    {

        if (!ctb::GetParamVector(confObj, gps_stdd, "gps_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, compass_stdd, "compass_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, magnetometer_stdd, "magnetometer_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, gyro_stdd, "gyro_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, accelerometer_stdd, "accelerometer_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, dvl_stdd, "dvl_stdd"))
            return false;
        if (!ctb::GetParamVector(confObj, orientus_stdd, "imu_orientus_stdd"))
            return false;
        if (!ctb::GetParam(confObj, fog_stdd, "fog_stdd"))
            return false;
        if (!ctb::GetParam(confObj, pressure_stdd, "pressure_stdd"))
            return false;

        const libconfig::Setting& gyro_bias = confObj["gyro_bias"];

        const libconfig::Setting& b_x = gyro_bias["bx"];
        if (!bx.ConfigureFromFile(b_x)) {
            std::cerr << "Failed to laod gyro bias on x" << std::endl;
            return false;
        }

        const libconfig::Setting& b_y = gyro_bias["by"];
        if (!by.ConfigureFromFile(b_y)) {
            std::cerr << "Failed to laod gyro bias on x" << std::endl;
            return false;
        }

        const libconfig::Setting& b_z = gyro_bias["bz"];
        if (!bz.ConfigureFromFile(b_z)) {
            std::cerr << "Failed to laod gyro bias on x" << std::endl;
            return false;
        }

        return true;
    }
};
*/
struct VisualizerConfiguration {
    //SensorsNoise sensorsNoise;
    int rate;
    /*
    double modelErrorPercentage;
    UnderwaterModelParameters ROVmodelParams;
    CableParameters CableROVmodelParams; // cable for ROV
    Eigen::Vector3d inertialF_waterCurrent; // 3d for ROV
    Eigen::Vector3d bodyF_gps_sensor_position;
    Eigen::Vector6d bodyF_dvl_sensor_pose;
    Eigen::Vector6d bodyF_imu_sensor_pose;
    Eigen::Vector6d bodyF_fog_sensor_pose;
    SinusoidalWave wx, wy, wz; // 3d for ROV
*/
    bool ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
    {
        if (!ctb::GetParam(confObj, rate, "rate"))
            return false;
        /*
        if (!ctb::GetParam(confObj, modelErrorPercentage, "modelErrorPercentage"))
            return false;
        if (!ctb::GetParamVector(confObj, inertialF_waterCurrent, "inertialF_waterCurrent"))
            return false;
        if (!ctb::GetParamVector(confObj, bodyF_gps_sensor_position, "bodyF_gps_sensor_position"))
            return false;
        if (!ctb::GetParamVector(confObj, bodyF_dvl_sensor_pose, "bodyF_dvl_sensor_pose"))
            return false;
        if (!ctb::GetParamVector(confObj, bodyF_imu_sensor_pose, "bodyF_imu_sensor_pose"))
            return false;
        if (!ctb::GetParamVector(confObj, bodyF_fog_sensor_pose, "bodyF_fog_sensor_pose"))
            return false;

        // ROV model parameters
        if (!ROVmodelParams.LoadConfiguration(confObj)) {
            RCLCPP_ERROR(rclcpp::get_logger("SimulatorConfiguration"), "Failed to load ROV model params");
            return false;
        }
        // ROV Cable model parameters
        if (!CableROVmodelParams.LoadConfiguration(confObj)) {
            RCLCPP_ERROR(rclcpp::get_logger("SimulatorConfiguration"), "Failed to load ROV Cable model params");
            return false;
        }

        // Add model error
        // construct a trivial random generator engine from a time-based seed:
        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> distribution(1.0 - modelErrorPercentage / 100, 1 + modelErrorPercentage / 100);
*/
        /*
        ROVmodelParams.m *= distribution(generator);
        ROVmodelParams.rho *= distribution(generator);
        ROVmodelParams.L *= distribution(generator);
        ROVmodelParams.H *= distribution(generator);
        ROVmodelParams.G *= distribution(generator);
        ROVmodelParams.B *= distribution(generator);
        for(int i=0; i<6; i++){
            ROVmodelParams.M_a_diag(i) *= distribution(generator);
            ROVmodelParams.D_diag(i) *= distribution(generator);
            ROVmodelParams.K_diag(i) *= distribution(generator);
        }
        for(int i=0; i<36; i++){
            ROVmodelParams.T_vector(i) *= distribution(generator);
        }
        for(int i=0; i<3; i++){
            ROVmodelParams.CG(i) *= distribution(generator);
            ROVmodelParams.CB(i) *= distribution(generator);
        }
        */
/*

        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& sensorsnoise = root["sensorsNoise"];
        if (!sensorsNoise.ConfigureFromFile(sensorsnoise)) {
            std::cerr << "Failed to load rov sensors noise " << std::endl;
            return false;
        }

        //load additive sinusoidal noise on wx wy to simulate wawes
        const libconfig::Setting& wavesSimulator = root["wavesSimulator"];
        const libconfig::Setting& omega_x = wavesSimulator["wx"];

        if (!wx.ConfigureFromFile(omega_x)) {
            std::cerr << "Failed to load waves params on x " << std::endl;
            return false;
        }

        const libconfig::Setting& omega_y = wavesSimulator["wy"];

        if (!wy.ConfigureFromFile(omega_y)) {
            std::cerr << "Failed to load waves params on y " << std::endl;
            return false;
        }

        const libconfig::Setting& omega_z = wavesSimulator["wz"];

        if (!wz.ConfigureFromFile(omega_z)) {
            std::cerr << "Failed to load waves params on z " << std::endl;
            return false;
        }
*/
        return true;
    }
};

namespace inputs {

namespace ID {

const uint8_t halt = 0;
const uint8_t hold = 5;
const uint8_t forward = 8;
const uint8_t backward = 2;
const uint8_t left = 6;
const uint8_t right = 4;
const uint8_t up = 9;
const uint8_t down = 3;
}
}

}

#endif // VISUALIZER_DEFINES_H
