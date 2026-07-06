# TDZK-T002

ROS 2 workspace for T002 humanoid robot deployment. Reuses the hardware
abstraction layer (motors + robot_hardware + imu) from TDZK-T001 and adds
a new STW motor driver.

## Environment

- **ROS 2 Humble** (Hawksbill)
- **Gazebo Ignition Fortress** (for simulation)
- colcon build system

```bash
source /opt/ros/humble/setup.bash
```

### Required system packages

```bash
sudo apt install -y python3-colcon-common-extensions ros-humble-xacro
```

For real-hardware deployment also install:
```bash
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-hardware-interface \
  ros-humble-joint-state-broadcaster
```

## Structure

```
src/
├── hardware/
│   ├── motors/           # CAN motor drivers (DM, EVO, ENCOS, STW)
│   ├── imu/              # IMU driver and serial protocol
│   └── robot_hardware/   # ros2_control SystemInterface plugin
├── controller/
│   └── t002_controller/  # T002-specific controller
urdfs/
└── T002_description/     # URDF/Xacro and ros2_control config
scripts/                  # Helper scripts
assets/                   # Static assets
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select motors imu robot_hardware t002_controller t002_description
source install/setup.bash
```

## Launch (real hardware / desktop debug)

```bash
# 1. Bring up CAN interfaces (done separately, not in this repo)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 2. Launch
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch t002_controller deploy.launch.py
```

## Gazebo / Ignition Simulation

If you need Gazebo Ignition support, create a `urdfs/T002_gz_description/` package
with an `ign_ros2_control/IgnitionSystem` hardware plugin (instead of
`robot_hardware/RobotHardwareNode`). The controller (t002_controller) and config
remain the same — only the hardware plugin differs between sim and real.

## Jazzy → Humble API Changes Applied

The following changes were already made to ensure Humble compatibility:

| File | Change |
|------|--------|
| `robot_hardware_node.hpp` | `HardwareComponentInterfaceParams` → `HardwareInfo` |
| `robot_hardware_node.hpp` | Include: `hardware_component_interface_params.hpp` → `hardware_info.hpp` |
| `t002_controller.cpp` | `iface.get_optional().value_or(0.0)` → `iface.get_value()` |

## Motor Drivers

| Driver | Brand | Control Modes | Protocol |
|--------|-------|---------------|----------|
| dm     | DM    | MIT, POS, SPD | CAN      |
| evo    | EVO   | MIT, POS, SPD | CAN      |
| encos  | ENCOS | MIT, POS, SPD | CAN      |
| stw    | STW   | MIT           | CAN      |
