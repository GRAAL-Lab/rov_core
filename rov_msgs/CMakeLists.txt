cmake_minimum_required(VERSION 3.8)
project(rov_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(fastrtps)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "srv/*.srv"
    "*.xml"
    "launch/*.py"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

rosidl_generate_interfaces(rov_msgs
  # Sensors
  "msg/Time.msg"
  "msg/RPY.msg"
  "msg/IMUData.msg"
  "msg/Compass.msg"
  "msg/Magnetometer.msg"
  "msg/PressureData.msg"
  "msg/LatLong.msg"
  "msg/LatLongAlt.msg"
  "msg/SimulatedSystem.msg"
  "msg/MicroLoopCount.msg"
  "msg/GPSData.msg"
  "msg/Forces.msg"
  "msg/ReferenceVelocities.msg"
  "msg/ThrustersReference.msg"
  "msg/DynamicPidControl.msg"
  "msg/VehicleStatus.msg"
  "msg/CableData.msg"
  "msg/CableLengthReference.msg"
  "msg/WinchMotorReference.msg"

  # Commands
  "msg/CommandHold.msg"
  "msg/CommandLatLongAlt.msg"
  "msg/CommandSpeedHeading.msg"
  
  # LLC
  #"msg/LLCAck.msg"

  # Control
  #"msg/PolygonPath.msg"
  "msg/NavFilterData.msg"
  "msg/TaskStatus.msg"
	
  

  ## Services ##
  "srv/UserInput.srv"
  "srv/ControlCommand.srv"

  
  ## Other ##

)

ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/
  DESTINATION include
)

#install(
#  # File
#  DESTINATION share/${PROJECT_NAME}
#)

ament_package()
