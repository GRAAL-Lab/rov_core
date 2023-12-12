/*
 * controller_console_node.cpp
 *
 *  Created on: Nov 01, 2018
 *      Author: francescow
 */

#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "rov_ctrl/rov_defines.hpp"

#include "rov_msgs/srv/user_input.hpp"
#include "rov_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

using namespace rov;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("controller_console_node");
    rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr serviceClient =
        node->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);

    //rclcpp::Client<rov_msgs::srv::UserInput>::SharedPtr cliUserInput_;

    int choice;
    bool send;

    //auto serviceClient = node->create_client<rov_msgs::srv::UserInput>(rov_msgs::topicnames::user_input_service);
    //this->create_client<rov_msgs::srv::UserInput>("user_input");
    while (!serviceClient->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for Controller service to appear...");
    }

    auto serviceReq = std::make_shared<rov_msgs::srv::UserInput::Request>();

    while (rclcpp::ok()) {
        std::cout << std::endl;
        std::cout << "1)  Halt ° " << std::endl;
        std::cout << "2)  Hold * " << std::endl;
        std::cout << "3)  Move Forward -)" << std::endl;
        std::cout << "4)  Move Backward (-" << std::endl;
        std::cout << "5)  Move Up ^ " << std::endl;
        std::cout << "6)  Move Down _ " << std::endl;
        std::cout << "7)  Turn Left <--" << std::endl;
        std::cout << "8)  Turn Right -->" << std::endl;

        //std::cout << tc::bluL << "4)  " << tc::none << "Speed-Heading reference" << std::endl;
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
            serviceReq->motion_type = rov::inputs::ID::halt;
            std::cout << "halt " <<std::endl;
        } break;
        case 2: {
            serviceReq->motion_type = rov::inputs::ID::hold;
            std::cout << "hold " <<std::endl;
        } break;
        case 3: {
            serviceReq->motion_type = rov::inputs::ID::forward;
            std::cout << "forward " <<std::endl;
        } break;
        case 4: {
            serviceReq->motion_type = rov::inputs::ID::backward;
            std::cout << "backward " <<std::endl;
        } break;
        case 5: {
            serviceReq->motion_type = rov::inputs::ID::up;
            std::cout << "up " <<std::endl;
        } break;
        case 6: {
            serviceReq->motion_type = rov::inputs::ID::down;
            std::cout << "down " <<std::endl;
        } break;
        case 7: {
            serviceReq->motion_type = rov::inputs::ID::left;
            std::cout << "left " <<std::endl;
        } break;
        case 8: {
            serviceReq->motion_type = rov::inputs::ID::right;
            std::cout << "right " <<std::endl;
        } break;

        default:
            std::cout << "Unsupported choice! " << choice << std::endl;
            send = false;
            continue;
            //break;
        }

        if (send) {
            auto result_future = serviceClient->async_send_request(serviceReq);
            std::cout << "Sent Request to controller" << std::endl;
            if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            } else {
                auto result = result_future.get();
                RCLCPP_INFO(node->get_logger(), "Service returned: %s", (result->res).c_str());
            }
        }
    }
}
