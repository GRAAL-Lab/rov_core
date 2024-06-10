# ROV Control

ROV controller revamped with ROS2.

## Subpackages

Short description of all the packages included in this meta package:

- **navigation_filter**: The sensor filtering package providing the vehicle status and seacurrent estimation to the controller.
- **rov_msgs**: Interface and services messages package with headers for topic names and common variables.
- **rov_ctrl**: The ROV controller.
- **rov_sim**: The dynamic simulator, which makes use of `underwater_vehicle_model` library or replays data from logs.
- **rov_map**: Graphical interface (Qt based) for controlling the ROV.


## Dependencies

In order of installation to respect dependencies:

- **ros2 galactic**: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
- **rml**: http://bitbucket.org/isme_robotics/rml
- **fsm**: http://bitbucket.org/isme_robotics/fsm
- **tpik**: http://bitbucket.org/isme_robotics/tpik
- **ikcl**: https://bitbucket.org/isme_robotics/ikcl
- **libgps**: `sudo apt install libgps-dev`
- **SISL lib**: `git clone https://github.com/SINTEF-Geometry/SISL.git`
- **ctrl_toolbox**: http://bitbucket.org/isme_robotics/ctrl_toolbox
- **sisl_toolbox**: http://bitbucket.org/isme_robotics/sisl_toolbox
- **qt5-libraries**: 

- **marine_vehicle_models**: https://bitbucket.org/isme_robotics/marine_vehicle_models/src/main/
