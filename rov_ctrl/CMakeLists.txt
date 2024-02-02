cmake_minimum_required(VERSION 3.5)
project(rov_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(surface_vehicle_model REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_driver REQUIRED)
find_package(rml REQUIRED)
find_package(fsm REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(tpik REQUIRED)
find_package(ikcl REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sisl_toolbox REQUIRED)

find_package(underwater_vehicle_model REQUIRED)
find_package(rov_msgs REQUIRED)

link_directories(${Boost_LIBRARIES})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES} sisl)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  #${ulisse_driver_INCLUDE_DIRS}
  ${ulisse_msgs_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}

  ${rov_msgs_INCLUDE_DIRS}
  ${underwater_vehicle_model_INCLUDE_DIRS}
)

set(use_SYSTEM_BOOST TRUE)

## Main Libraries and depedencies ##
set(rov_kinematic_ctrl_lib_sources
    # Main Classes
    src/rov_ctrl/kinematic_rov_controller.cpp
    # States
    src/rov_ctrl/states/generic_state.cpp
    src/rov_ctrl/states/state_halt.cpp
    src/rov_ctrl/states/state_latlong.cpp
    src/rov_ctrl/states/state_hold.cpp
    src/rov_ctrl/states/state_surgeyawrate.cpp
    # Commands
    src/rov_ctrl/commands/generic_command.cpp
    src/rov_ctrl/commands/command_halt.cpp
    src/rov_ctrl/commands/command_hold.cpp
    src/rov_ctrl/commands/command_latlong.cpp
    src/rov_ctrl/commands/command_surgeyawrate.cpp
    # Events
    #src/ulisse_ctrl/events/generic_event.cpp
    src/rov_ctrl/events/event_rc_enabled.cpp
    src/rov_ctrl/events/event_near_goal_position.cpp
    # TASKS
    src/rov_ctrl/tasks/SafetyBoundaries.cpp

)

set(rov_dynamic_ctrl_lib_sources
    # Main Class
    src/rov_ctrl/dynamic_rov_controller.cpp
)

## Libraries
add_library(rov_kinematic_ctrl_lib ${rov_kinematic_ctrl_lib_sources})
ament_target_dependencies(rov_kinematic_ctrl_lib rclcpp std_msgs ulisse_msgs surface_vehicle_model ament_index_cpp)
target_link_libraries(rov_kinematic_ctrl_lib ${surface_vehicle_model_LIBRARIES} ${ulisse_msgs_LIBRARIES} ${underwater_vehicle_model_LIBRARIES} ${rov_msgs_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ikcl tpik config++ Geographic rml fsm ctrl_toolbox sisl_toolbox)

add_library(rov_dynamic_ctrl_lib ${rov_dynamic_ctrl_lib_sources})
ament_target_dependencies(rov_dynamic_ctrl_lib rclcpp std_msgs rov_msgs underwater_vehicle_model ament_index_cpp)
target_link_libraries(rov_dynamic_ctrl_lib ${underwater_vehicle_model_LIBRARIES} ${rov_msgs_LIBRARIES} config++ Geographic rml fsm ctrl_toolbox)


add_executable(kinematic_control_node src/kinematic_control_node.cpp)
ament_target_dependencies(kinematic_control_node rclcpp std_msgs rov_msgs rml)
target_link_libraries(kinematic_control_node rml rov_kinematic_ctrl_lib ${JSONCPP_LIBRARIES})

add_executable(controller_console_node src/controller_console_node.cpp)
ament_target_dependencies(controller_console_node rclcpp std_msgs rov_msgs rml)
target_link_libraries(controller_console_node rml rov_kinematic_ctrl_lib)

add_executable(dynamic_control_node src/dynamic_control_node.cpp)
ament_target_dependencies(dynamic_control_node rclcpp std_msgs rov_msgs rml ament_index_cpp ctrl_toolbox)
target_link_libraries(dynamic_control_node rml rov_dynamic_ctrl_lib)

ament_export_include_directories(include)

## Install
install(TARGETS
  kinematic_control_node
  dynamic_control_node
  controller_console_node
  #monitor_node
  #control_test
  DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY
  #launch
  conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
