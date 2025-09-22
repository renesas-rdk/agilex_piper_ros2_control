# agilex_piper_ros2_control

ROS 2 package that provides a ros2_control hardware interface for the AgileX Piper arm and gripper. This package contains the hardware interface implementation that communicates with the physical robot hardware.

## Features
- ros2_control hardware interface plugin (exported via `hardware_interface_plugin.xml`)
- Hardware interface implementation for CAN communication with the robot
- ros2_control URDF macro for hardware interface integration
- **GPIO controller integration** for extended arm features (enable/disable, pose feedback, status monitoring)

## GPIO Controller Extensions
This package now includes GPIO controller interfaces, providing access to:
- **arm_admin**: Administrative control (enable/disable arm, connection status)
- **arm_current_pose**: End effector pose feedback (read-only monitoring)
- **arm_target_pose**: Target pose commands (Cartesian mode only, command-only)
- **arm_status**: Comprehensive arm status monitoring (control mode, errors, teaching status)
- **arm_motion_mode**: Motion mode and speed control (built-in arm feature)

## Related Packages
- **agilex_piper_arm_bringup**: Contains launch files, controller configurations, robot URDF descriptions, and test scripts for running the robot
- **agilex_piper_arm_description**: Contains the robot's visual and collision meshes, joint definitions

## Package layout
- `include/` / `src/`: Hardware interface implementation (`agilex_piper_hardware_interface`)
- `config/`: GPIO controller configuration (`agilex_piper_gpio_controller.yaml`)
- `urdf/`: ros2_control URDF macro for hardware interface integration
- `hardware_interface_plugin.xml`: Plugin description file for the hardware interface

## Prerequisites
- ROS 2 (Jazzy or newer) with `ros2_control` ecosystem
- A colcon workspace (e.g., `~/ros2_ws`)
- CAN interface support for hardware communication
- `agilex_piper_controller` package for low-level robot communication

## Usage
This package provides the hardware interface that should be used with the `agilex_piper_arm_bringup` package for launching and controlling the robot.

To use this hardware interface in your robot system, you'll typically launch one of the bringup configurations:

```bash
# For joint trajectory control
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_trajectory_control.launch.py

# For joint position control
ros2 launch agilex_piper_arm_bringup agilex_piper_joint_position_control.launch.py

# For Cartesian motion control
ros2 launch agilex_piper_arm_bringup agilex_piper_cartesian_motion_control.launch.py
```

After launching, you can introspect the hardware interface:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## URDF / Xacro
The ros2_control hardware interface macro is provided in `urdf/`:
- `agilex_piper_macro.ros2_control.xacro`: ros2_control hardware interface, transmissions, and interfaces

To include the hardware interface in your robot URDF:
```xml
<xacro:include filename="$(find agilex_piper_ros2_control)/urdf/agilex_piper_macro.ros2_control.xacro"/>
<xacro:agilex_piper_ros2_control
    name="agilex_piper_arm"
    can_interface="can0"
    use_mock_hardware="false"
    include_gripper="true"
    prefix=""/>
```
Adjust arguments as needed (see the xacro file for available parameters).

## Configuration Options
The hardware interface supports the following configuration options:
- `can_interface`: CAN interface for hardware communication (e.g., "can0", "can1")
- `use_mock_hardware`: Set to "true" for simulation/testing without physical hardware
- `include_gripper`: Set to "true" to include gripper interfaces
- `prefix`: Namespace prefix for joint names (optional)

### GPIO Controller Usage
To use the GPIO controller features, load the GPIO controller configuration:
```bash
# Load GPIO controller via controller manager
ros2 run controller_manager spawner agilex_piper_gpio_controller --controller-manager /controller_manager
```

## Development
For detailed usage examples, launch configurations, controller setups, and test scripts, see the `agilex_piper_arm_bringup` package.

## License and maintainers
Refer to `package.xml` for license and maintainer information.
