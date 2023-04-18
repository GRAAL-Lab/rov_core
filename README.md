# ROV Control

ROV controller revamped with ROS2.

## Subpackages

Short description of all the packages included in this meta package:

- **navigation_filter**: The sensor filtering package providing the vehicle status and seacurrent estimation to the controller.
- **rov_msgs**: Interface and services messages package with headers for topic names and common variables.
- **rov_ctrl**: The catamaran controller.
- **rov_sim**: The dynamic simulator, which makes use of `surface_vehicle_model` library or replays data from logs.
- **rov_map**: Graphical interface (Qt based) for controlling the catamaran.

