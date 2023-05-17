cmake_minimum_required(VERSION 3.8)
project(nav_filter_rov)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  #${rov_msgs_INCLUDE_DIRS}
)

## Libraries ##
set(nav_filter_rov_lib_sources
    # Main Classes
    src/${PROJECT_NAME}/navigation_filter_rov.cpp
    #src/${PROJECT_NAME}/luenberger_observer/pos_vel_observer.cpp
    # Helpers
    #src/${PROJECT_NAME}/nav_data_structs.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/gps.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/compass.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/magnetometer.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/accelerometer.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/gyro.cpp
    #src/${PROJECT_NAME}/kalman_filter/ulisse_vehicle_model.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/z_meter.cpp
    #src/${PROJECT_NAME}/kalman_filter/measurements/rpm.cpp
)

## Libraries
add_library(nav_filter_rov_lib ${nav_filter_rov_lib_sources})
ament_target_dependencies(nav_filter_rov_lib rclcpp std_msgs rov_msgs underwater_vehicle_model)
target_link_libraries(nav_filter_rov_lib ${underwater_vehicle_model_LIBRARIES} Geographic rml) # !!fsm ctrl_toolbox

# add_library(surface_vehicle_model_lib src/surfacevehiclemodel.cpp)
# target_link_libraries(surface_vehicle_model_lib rml)

## Install
install(DIRECTORY
#  launch
  conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS nav_filter_rov_lib # foo_bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(nav_filter_rov_lib)

ament_package()
