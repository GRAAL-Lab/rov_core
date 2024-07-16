cmake_minimum_required(VERSION 3.8)
project(rov_vis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
find_package(visualization_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${rov_msgs_INCLUDE_DIRS}
  ${rov_ctrl_INCLUDE_DIRS}
  ${underwater_vehicle_model_INCLUDE_DIRS}
)

## Libraries
add_library(VehicleVisualizerClass src/rov_vis/rov_visualizer.cpp)
ament_target_dependencies(
    VehicleVisualizerClass
    rclcpp
    rov_msgs
    #ulisse_msgs
    rov_ctrl
    ament_index_cpp
    geometry_msgs
    tf2
    tf2_ros
    visualization_msgs
    ) #add rov message
target_link_libraries(VehicleVisualizerClass ${underwater_vehicle_model_LIBRARIES} Geographic rml)

## Executables
add_executable(visualizer_node src/visualizer_node.cpp)
ament_target_dependencies(
    visualizer_node
    rclcpp std_msgs
    rov_msgs
    #ulisse_msgs
    #rov_ctrl
    #underwater_vehicle_model
    )
target_link_libraries(visualizer_node VehicleVisualizerClass config++)

install(TARGETS
  visualizer_node
  #sim_from_log_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  #logs
  #launch
#  meshes
  conf
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
