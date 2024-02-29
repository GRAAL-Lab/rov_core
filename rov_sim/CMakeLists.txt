cmake_minimum_required(VERSION 3.8)
project(rov_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(underwater_vehicle_model REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(rov_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(rov_ctrl REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "logs/*.log"
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${rov_msgs_INCLUDE_DIRS}
  ${rov_ctrl_INCLUDE_DIRS}
  ${underwater_vehicle_model_INCLUDE_DIRS}
)

## Libraries
add_library(VehicleSimulatorClass src/rov_sim/rov_simulator.cpp)
ament_target_dependencies(
    VehicleSimulatorClass
    rclcpp
    rov_msgs
    ulisse_msgs
    rov_ctrl
    ament_index_cpp
    geometry_msgs
    tf2
    tf2_ros
    ) #add rov message
target_link_libraries(VehicleSimulatorClass ${underwater_vehicle_model_LIBRARIES} Geographic rml)

## Executables
add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(
    simulator_node
    rclcpp std_msgs
    rov_msgs
    ulisse_msgs
    rov_ctrl
    underwater_vehicle_model
    )
target_link_libraries(simulator_node VehicleSimulatorClass config++)

install(TARGETS
  simulator_node
  #sim_from_log_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  #logs
  #launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
