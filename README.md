# agilex_piper_ros2_control

ROS 2 package that provides a ros2_control hardware interface, controller configurations, URDF/Xacro descriptions, and launch files for the AgileX Piper arm and gripper.

## Features
- ros2_control hardware interface plugin (exported via `hardware_interface_plugin.xml`).
- URDF/Xacro for the arm, gripper, and ros2_control tags.
- Controller configurations for joint trajectory, joint position, cartesian motion, and gripper control.
- Launch files to start controller_manager and load controllers.
- Example Python script to send a simple joint trajectory.
- Optional Foxglove Studio layout.

## Package layout
- `include/` / `src/`: Hardware interface implementation (`agilex_piper_hardware_interface`).
- `urdf/`: Xacro files for the arm, gripper, and ros2_control integration.
- `config/`: Controller and controller_manager YAML configuration.
- `launch/`: Launch files for different control modes.
- `test/`: Simple trajectory test script.

## Prerequisites
- ROS 2 (Jazzy or newer) with `ros2_control` and `ros2_controllers` ecosystem.
- A colcon workspace (e.g., `~/ros2_ws`).

## Launch
Start controller manager and the configured controllers for different control modes:

- Joint trajectory control (FollowJointTrajectory action)
```bash
ros2 launch agilex_piper_ros2_control agilex_piper_joint_trajectory_control.launch.py
```

- Joint position control (direct command interface)
```bash
ros2 launch agilex_piper_ros2_control agilex_piper_joint_position_control.launch.py
```

- Cartesian motion control
```bash
ros2 launch agilex_piper_ros2_control agilex_piper_cartesian_motion_control.launch.py
```

After launching, introspect controllers and hardware:
```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

## Detailed usage and modes
- For command-line examples, arguments, and usage tips, refer to the header comments at the top of each launch file in `launch/`.
- Both physical robot and simulation/testing configurations are supported:
  - Physical robot: run defaults or set `can_interface:=canX` and `include_gripper:=true|false` as needed.
  - Simulation/testing: set `use_mock_hardware:=true` to run without hardware.

## Controllers
Controller configurations live under `config/`:
- `controller_manager.yaml`: Controller manager settings and controller load configuration.
- `agilex_piper_joint_trajectory_controller.yaml`: Joint trajectory controller parameters.
- `agilex_piper_joint_position_controller.yaml`: Joint position (forward command) controller parameters.
- `agilex_piper_gripper_action_controller.yaml`: Gripper command controller parameters.
- `agilex_piper_cartesian_motion_controller.yaml`: Cartesian motion controller parameters.
- `agilex_piper_motion_control_handle.yaml`: Motion control handle configuration (if used by the cartesian controller).

Controller names and exact parameters are defined in the YAML files and loaded by the launch files.

## URDF / Xacro
Xacros are provided in `urdf/`:
- `agilex_piper_arm.urdf.xacro` and `agilex_piper_arm_gripper.urdf.xacro`: Robot description for arm and gripper.
- `agilex_piper_macro.ros2_control.xacro`: ros2_control hardware, transmissions, and interfaces.

To include in your own robot:
```xml
<xacro:include filename="$(find-pkg-share agilex_piper_ros2_control)/urdf/agilex_piper_arm.urdf.xacro"/>
<xacro:agilex_piper_arm/>
```
Adjust arguments as needed (see Xacro files for available parameters).

## Example: send a trajectory
With the trajectory controller running:
```bash
# Terminal 1
ros2 launch agilex_piper_ros2_control agilex_piper_joint_trajectory_control.launch.py

# Terminal 2
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robots/agilex_piper_arm/agilex_piper_ros2_control/test/test_joint_trajectory.py
```
The script publishes a simple trajectory to the configured `joint_trajectory_controller`.

## Foxglove Studio
An optional layout is available at `config/foxglove/arm_ros2_control.json`. Import it into Foxglove Studio to visualize topics (joint states, controller feedback, etc.).

## License and maintainers
Refer to `package.xml` for license and maintainer information.
