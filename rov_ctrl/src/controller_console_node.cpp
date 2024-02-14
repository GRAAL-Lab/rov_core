/*
 * controller_console_node.cpp
 *
 */

#include <cstdio>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "rov_ctrl/rov_defines.hpp"

#include "rov_msgs/srv/control_command.hpp"
#include "rov_msgs/srv/user_input.hpp"
#include "rov_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
//#include "GeographicLib/UTMUPS.hpp"ù
#include "GeographicLib/Geodesic.hpp"
//#include "rml/RML.h"
#include <rml/RML.h>
#include "rov_ctrl/ctrl_data_structs.hpp"

using namespace rov;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("controller_console_node");
    rclcpp::Client<rov_msgs::srv::ControlCommand>::SharedPtr serviceClient =
        node->create_client<rov_msgs::srv::ControlCommand>(rov_msgs::topicnames::control_cmd_service);
    rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr serviceClient2 =
        node->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);

    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr cliUserInput_;

    int choice;
    int direction;
    bool send, send2;

    //auto serviceClient = node->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);
    //this->create_client<rov_msgs::srv::UserInput>("user_input");

    while (!serviceClient->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for Controller service to appear...");
    }

    while (!serviceClient2->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for UserInput service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for Controller UserInput service to appear...");
    }
    auto serviceReq = std::make_shared<rov_msgs::srv::ControlCommand::Request>();
    auto serviceReq2 = std::make_shared<rov_msgs::srv::UserInput::Request>();

    while (rclcpp::ok()) {
        while(choice == 4){

            std::cout << std::endl;

            std::cout << "(5)  Hold * " << std::endl;
            std::cout << "(8)  Move Forward -)" << std::endl;
            std::cout << "(2)  Move Backward (-" << std::endl;
            std::cout << "(4)  Move Left <--" << std::endl;
            std::cout << "(6)  Move Right -->" << std::endl;
            std::cout << "(9)  Move Up ^ " << std::endl;
            std::cout << "(3)  Move Down _ " << std::endl;
            std::cout << "(7)  Turn Left <-(" << std::endl;
            std::cout << "(1)  Turn Right )->" << std::endl;
            std::cout << "(0)  back... " << std::endl;


            //std::cout << tc::bluL << "4)  " << tc::none << "Speed-Heading reference" << std::endl;
            std::cout << "Enter direction..." << std::endl;
            std::cin >> direction;

            send2 = true;

            switch (direction) {
            case 0: {
                serviceReq2->motion_type = rov::inputs::ID::halt;
                choice = 1;
                std::cout << "<- back " <<std::endl;
            } break;
            case 5: {
                serviceReq2->motion_type = rov::inputs::ID::hold;
                //choice = 2;
                std::cout << "hold " <<std::endl;
            } break;
            case 8: {
                serviceReq2->motion_type = rov::inputs::ID::forward;
                std::cout << "forward " <<std::endl;
            } break;
            case 2: {
                serviceReq2->motion_type = rov::inputs::ID::backward;
                std::cout << "backward " <<std::endl;
            } break;
            case 4: {
                serviceReq2->motion_type = rov::inputs::ID::left;
                std::cout << "left " <<std::endl;
            } break;
            case 6: {
                serviceReq2->motion_type = rov::inputs::ID::right;
                std::cout << "right " <<std::endl;
            } break;
            case 9: {
                serviceReq2->motion_type = rov::inputs::ID::up;
                std::cout << "up " <<std::endl;
            } break;
            case 3: {
                serviceReq2->motion_type = rov::inputs::ID::down;
                std::cout << "down " <<std::endl;
            } break;
            case 7: {
                serviceReq2->motion_type = rov::inputs::ID::turn_left;
                std::cout << "turn left " <<std::endl;
            } break;
            case 1: {
                serviceReq2->motion_type = rov::inputs::ID::turn_right;
                std::cout << "turn right " <<std::endl;
            } break;

            default:
                std::cout << "Unsupported choice! " << choice << std::endl;
                send2  = false;
                continue;
                //break;
            }
            if (send2) {
                auto result_future = serviceClient2->async_send_request(serviceReq2);
                std::cout << "Sent Request to UserInput controller" << std::endl;
                if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
                    RCLCPP_ERROR(node->get_logger(), "UserInput service call failed :(");
                    std::cout << "No response" << std::endl;
                } else {
                    auto result = result_future.get();
                    RCLCPP_INFO(node->get_logger(), "UserInput Service returned: %s", (result->res).c_str());
                    std::cout << "Response Recieved" << std::endl;
                }
            }


        }
        std::cout << std::endl;
        std::cout << "1)  " << "Halt" << std::endl;
        std::cout << "2)  " << "Hold Position" << std::endl;
        std::cout << "3)  " << "Move to Lat-Long" << std::endl;
        std::cout << "4)  " << "Speed-Heading reference" << std::endl;
        std::cout << "Enter command..." << std::endl;
        std::cin >> choice;

        if (std::cin.fail()) {
            std::cout << "Flushing bad input!" << std::endl;
            std::cin.clear(); // unset failbit
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        send = true;

        switch (choice) {
        case 1: {
            serviceReq->command_type= rov::commands::ID::halt;
        } break;
        case 2: {
            serviceReq->command_type = rov::commands::ID::hold;
            //std::cout << "acceptanceRadius ";
            //std::cin >> serviceReq->hold_cmd.acceptance_radius;
        } break;
        case 3: {
            serviceReq->command_type = rov::commands::ID::latlongalt;
            /*
            std::cout << "latitude [m]: ";
            std::cin >> serviceReq->moveto_cmd.goal.latlong.latitude;
            std::cout << "longitude [m]: ";
            std::cin >> serviceReq->moveto_cmd.goal.latlong.longitude;
            std::cout << "altitude [m]: ";
            std::cin >> serviceReq->moveto_cmd.goal.altitude;
            //std::cout << "acceptanceRadius: ";
            //std::cin >> serviceReq->moveto_cmd.acceptance_radius;
            */
            Eigen::Vector3d goal_cartesian;
            std::cout << "x [m]: ";
            std::cin >> goal_cartesian.x();
            std::cout << "y [m]: ";
            std::cin >> goal_cartesian.y();
            std::cout << "z [m]: ";
            std::cin >> goal_cartesian.z();

            ctb::LatLong centroidLocation_;
            centroidLocation_.latitude = 44.0956;
            centroidLocation_.longitude = 9.8631;
            ctb::LatLong goal_latlong; double goal_altitude;

            ctb::LocalUTM2LatLong(goal_cartesian, centroidLocation_, goal_latlong, goal_altitude);
            serviceReq->moveto_cmd.goal.latlong.latitude = goal_latlong.latitude;
            serviceReq->moveto_cmd.goal.latlong.longitude = goal_latlong.longitude;
            serviceReq->moveto_cmd.goal.altitude = goal_altitude;

        } break;
        case 4: {
            serviceReq->command_type = rov::commands::ID::velocity;

            std::cout << "speed_x [m/s]: ";
            std::cin >> serviceReq->sh_cmd.speed[0];
            std::cout << "speed_y [m/s]: ";
            std::cin >> serviceReq->sh_cmd.speed[1];
            std::cout << "speed_z [m/s]: ";
            std::cin >> serviceReq->sh_cmd.speed[2];

            std::cout << "heading speed [rad/s]: ";
            std::cin >> serviceReq->sh_cmd.heading;

            //std::cout << "timeout [s] ";
            //std::cin >> serviceReq->sh_cmd.timeout.sec;
            serviceReq->sh_cmd.timeout.sec = 10;
        } break;
        default:
            std::cout << "Unsupported choice! " << choice << std::endl;
            send = false;
            continue;
            //break;
        }

        if (send) {
            auto result_future = serviceClient->async_send_request(serviceReq);
            std::cout << "Sent Request to ControlCommand controller" << std::endl;
            if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "ControlCommand service call failed :(");
            } else {
                auto result = result_future.get();
                RCLCPP_INFO(node->get_logger(), "ControlCommand Service returned: %s", (result->res).c_str());
            }
        }
    }
}
