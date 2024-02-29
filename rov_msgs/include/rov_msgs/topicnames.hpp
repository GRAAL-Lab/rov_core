#ifndef TOPICNAMES_HPP
#define TOPICNAMES_HPP

#include <string>

namespace rov_msgs {

namespace topicnames {

// LLC
const std::string micro_loop_count = "/rov/llc/micro_loop_count";
const std::string sensor_gps_data = "/rov/llc/sensor/gps_data";
//const std::string sensor_gps_status = "/ulisse/llc/sensor/gps_status";
const std::string sensor_compass = "/rov/llc/sensor/compass";
const std::string sensor_imu = "/rov/llc/sensor/imu";
//const std::string sensor_ambient = "/ulisse/llc/sensor/ambient";
const std::string sensor_magnetometer = "/rov/llc/sensor/magnetometer";
const std::string sensor_pressure = "/rov/llc/sensor/pressure";

//const std::string sensor_imu_orientus = "/ulisse/llc/sensor/imu_orientus";

//const std::string sensor_dvl = "/ulisse/llc/sensor/dvl";
//const std::string sensor_fog = "/ulisse/llc/sensor/fog";
const std::string llc_thrusters_reference_perc = "/rov/llc/reference_thrusters_percentage"; // thrusters_data
//const std::string llc_thrusters_applied_perc = "/ulisse/llc/applied_thrusters_percentage";     // motor_applied_ref
//const std::string llc_status = "/ulisse/llc/status";
//const std::string llc_config = "/ulisse/llc/config";
//const std::string llc_thrusters = "/ulisse/llc/thrusters"; //llc_motors
//const std::string llc_version = "/ulisse/llc/version";
//const std::string llc_ack = "/ulisse/llc/ack";
//const std::string llc_battery_left = "/ulisse/llc/battery_left";
//const std::string llc_battery_right = "/ulisse/llc/battery_right";
//const std::string llc_sw485status = "/ulisse/llc/sw485status";

// SIM
const std::string simulated_system = "/rov/simulated_system";
const std::string forces = "/rov/forces";
const std::string cable_data = "/winch/cable_data";
const std::string reference_cable_length = "/winch/reference_cable_length";

// NAV FILTER
const std::string nav_filter_data = "/rov/nav_filter/data";

// CTRL
const std::string vehicle_status = "/rov/ctrl/vehicle_status";
//const std::string thruster_mapping_control = "/ulisse/ctrl/thruster_mapping_info";
const std::string classic_pid_control = "/rov/ctrl/classic_pid_control_info";
//const std::string computed_torque_control = "/ulisse/ctrl/computed_torque_control_info";
//const std::string sliding_mode_control = "/ulisse/ctrl/sliding_mode_info";
const std::string reference_velocities = "/rov/ctrl/reference_velocities";
//const std::string simulated_velocity_sensor = "/ulisse/ctrl/water_relative_surge";
//const std::string feedback_gui = "/ulisse/ctrl/feedback_gui";
//const std::string set_cruise_control_service = "/ulisse/ctrl/set_cruise_control";
//const std::string reset_kcl_conf_service = "/ulisse/ctrl/reset_kcl_configuration";
//const std::string reset_dcl_conf_service = "/ulisse/ctrl/reset_dcl_configuration";
//const std::string surge_heading = "/ulisse/ctrl/surge_heading";
//const std::string surge_yawrate = "/ulisse/ctrl/surge_yawrate";
//const std::string pathfollowing = "/ulisse/ctrl/pathfollowing"; // ILOS

// TASKS
const std::string task_absolute_axis_alignment = "/rov/task/ROV_Absolute_Axis_Alignment";
const std::string task_linear_velocity = "/rov/task/ROV_Linear_Velocity";
const std::string task_angular_velocity = "/rov/task/ROV_Angular_Velocity";
const std::string task_angular_position = "/rov/task/ROV_Angular_Position";
const std::string task_cartesian_distance = "/rov/task/ROV_Cartesian_Distance";
//const std::string task_cartesian_distance_path_follow = "/rov/task/ROV_Cartesian_Distance_Path_Follow";
const std::string task_safety_boundaries = "/rov/task/ROV_Safety_Boundaries";
const std::string task_absolute_axis_alignment_safety = "/rov/task/ROV_Absolute_Axis_Alignment_Safety";
//const std::string task_absolute_axis_alignment_hold = "/rov/task/ROV_Absolute_Axis_Alignment_Hold";
const std::string task_linear_velocity_hold = "/rov/task/ROV_Linear_Velocity_Hold";
const std::string task_cartesian_distance_hold = "/rov/task/ROV_Cartesian_Distance_Hold";

//const std::string tpik_action = "/ulisse/task/tpik_action";

// SERVICES
//const std::string rosbag_service = "/bag_recorder_client";
//const std::string llc_cmd_service = "/ulisse/service/llc_cmd";
//
//const std::string navfilter_cmd_service = "/ulisse/service/navfilter_cmd";
//const std::string set_boundaries_service = "/ulisse/ctrl/set_boundaries";
//const std::string get_boundaries_service = "/ulisse/ctrl/get_boundaries";
const std::string control_cmd_service = "/rov/service/control_cmd";
const std::string user_input_service = "/rov/service/user_input";

// OTHER
//const std::string obstacle = "/ulisse/ctrl/obstacle";
//const std::string avoidance_path = "/ulisse/ctrl/avoidance_path";
const std::string tf = "/rov/tf";
const std::string posROV = "/rov/posROV";
}
}

#endif // TOPICNAMES_HPP
