# ROV Control

A ROS 2 meta-package providing full-stack control software for a BlueROV2 (standard and heavy configurations). It includes navigation filtering, kinematic and dynamic control, simulation, and visualisation nodes.

## Packages

  - **nav_filter_rov**: Navigation filter — fuses IMU, GPS, DVL, compass, pressure and FOG sensor data to produce vehicle state estimates and sea current estimates consumed by the controllers.
  - **rov_msgs**: Shared ROS 2 message and service definitions used across all packages (e.g. `VehicleStatus`, `NavFilterData`, `Forces`, `ControlCommand`).
  - **rov_ctrl**: ROV controllers — exposes a *kinematic* control node (`kinematic_control_node`, TPIK-based) and a *dynamic* control node (`dynamic_control_node`) that compute thruster references from high-level commands.
  - **rov_sim**: Dynamic simulator — integrates the BlueROV2 hydrodynamic model (via `marine_vehicle_models`) at up to 100 Hz, and can also replay pre-recorded sensor logs.
  - **rov_vis**: Visualisation node — publishes vehicle pose and geometry markers to RViz and optionally bridges with the Stonefish simulator.

The external package  **marine_vehicle_models** must be added to the src of the colcon workspace: https://github.com/GRAAL-Lab/marine_vehicle_models

## Dependencies

  - **ROS 2 Humble** (Ubuntu 22.04): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Install in the order listed below to respect build dependencies:

  - **libconfig++**: `sudo apt install libgps-dev libconfig++-dev`
  - **SISL**: https://github.com/SINTEF-Geometry/SISL
  - **rml**: https://github.com/GRAAL-Lab/rml
  - **fsm**: https://github.com/GRAAL-Lab/fsm
  - **tpik**: https://github.com/GRAAL-Lab/tpik
  - **ikcl**: https://github.com/GRAAL-Lab/ikcl
  - **ctrl_toolbox**: https://github.com/GRAAL-Lab/ctrl_toolbox
  - **sisl_toolbox**: https://github.com/GRAAL-Lab/sisl_toolbox


## Building

```bash
# From the root of your ROS 2 workspace
colcon build --symlink-install
source install/setup.bash
```

## Running

### Launch all control nodes

The provided launch file starts the kinematic controller, dynamic controller, and navigation filter together:

```bash
ros2 launch rov_ctrl launchControl.py
```

### Run nodes individually

```bash
# Navigation filter
ros2 run nav_filter_rov nav_filter_node

# Kinematic controller (TPIK-based, reads kcl_rov.conf)
ros2 run rov_ctrl kinematic_control_node

# Dynamic controller (reads dcl_rov.conf)
ros2 run rov_ctrl dynamic_control_node

# Simulator (reads simulator_rov.conf)
ros2 run rov_sim simulator_node

# Visualiser (publishes to RViz)
ros2 run rov_vis visualizer_node
```

## Configuration

Each node loads a `.conf` file at startup (libconfig++ format). Default configuration files are located in the `conf/` subdirectory of each package and are installed to the package's share directory by the build system.

| Node | Config file |
|------|-------------|
| `kinematic_control_node` | `rov_ctrl/conf/kcl_rov.conf` |
| `dynamic_control_node` | `rov_ctrl/conf/dcl_rov.conf` |
| `simulator_node` | `rov_sim/conf/simulator_rov.conf` |
| `nav_filter_node` | `nav_filter_rov/conf/navigation_filter_rov.conf` |

Key simulator parameters (in `simulator_rov.conf`): vehicle model selection (`heavyConf`), sensor noise levels, water current, simulation rate (default 100 Hz).
