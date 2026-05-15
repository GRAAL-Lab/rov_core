# ROV Control

A ROS 2 meta-package providing full-stack control software for a BlueROV2 (standard and heavy configurations). It includes navigation filtering, kinematic and dynamic control, simulation, and visualisation nodes.

## Packages

| Package | Description |
|---------|-------------|
| **nav_filter_rov** | Navigation filter — fuses IMU, GPS, DVL, compass, pressure and FOG sensor data to produce vehicle state estimates and sea current estimates consumed by the controllers. |
| **rov_msgs** | Shared ROS 2 message and service definitions used across all packages (e.g. `VehicleStatus`, `NavFilterData`, `Forces`, `ControlCommand`). |
| **rov_ctrl** | ROV controllers — exposes a *kinematic* control node (`kinematic_control_node`, TPIK-based) and a *dynamic* control node (`dynamic_control_node`) that compute thruster references from high-level commands. |
| **rov_sim** | Dynamic simulator — integrates the BlueROV2 hydrodynamic model (via `marine_vehicle_models`) at up to 100 Hz, and can also replay pre-recorded sensor logs. |
| **rov_vis** | Visualisation node — publishes vehicle pose and geometry markers to RViz and optionally bridges with the Stonefish simulator. |

## Dependencies

Install in the order listed below to respect build dependencies.

### ROS 2

- **ROS 2 Galactic** (Ubuntu 20.04): https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
  > ⚠️ ROS 2 Galactic reached end-of-life in December 2022. Consider migrating to a supported distribution such as [Humble](https://docs.ros.org/en/humble/Installation.html) or [Jazzy](https://docs.ros.org/en/jazzy/Installation.html).

### System libraries

```bash
sudo apt install libgps-dev libconfig++-dev
```

### External ROS 2 / C++ packages

Clone each repository into your ROS 2 workspace `src/` directory and build with `colcon`.

| Package | URL |
|---------|-----|
| **rml** | http://bitbucket.org/isme_robotics/rml |
| **fsm** | http://bitbucket.org/isme_robotics/fsm |
| **tpik** | http://bitbucket.org/isme_robotics/tpik |
| **ikcl** | https://bitbucket.org/isme_robotics/ikcl |
| **ctrl_toolbox** | http://bitbucket.org/isme_robotics/ctrl_toolbox |
| **sisl_toolbox** | http://bitbucket.org/isme_robotics/sisl_toolbox |
| **marine_vehicle_models** | https://bitbucket.org/isme_robotics/marine_vehicle_models |
| **SISL** | https://github.com/SINTEF-Geometry/SISL |

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
