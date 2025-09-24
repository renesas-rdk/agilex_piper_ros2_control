// ********************************************************************************************************************
// Copyright [2025] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
//
// The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
// and/or its licensors ("Renesas") and subject to statutory and contractual protections.
//
// Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
// display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
// purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
// SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
// NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
// INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
// OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
// be subject to different terms.
// ********************************************************************************************************************
#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "agilex_piper_controller/piper_controller.hpp"
#include "agilex_piper_ros2_control/visibility_control.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace agilex_piper_ros2_control
{

class AgilexPiperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AgilexPiperHardwareInterface)

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  AGILEX_PIPER_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Convert between hardware-specific units (0.001 deg) and ROS standard units (rad)
  double hw_units_to_rad(int hw_units) const;
  int rad_to_hw_units(double rad) const;

  // Convert between hardware-specific gripper units and ROS standard units
  double hw_gripper_units_to_meters(int hw_units) const;
  int meters_to_hw_gripper_units(double meters) const;
  double hw_gripper_effort_units_to_nm(uint16_t hw_units) const;
  uint16_t nm_to_hw_gripper_effort_units(double nm) const;

  // Gripper control helper methods
  void update_gripper_positions_from_api(double api_position);

  // GPIO helper methods
  void initialize_gpio_interfaces();
  void update_gpio_states();
  void apply_gpio_commands();
  double hw_pose_units_to_meters(int hw_units) const;
  int meters_to_hw_pose_units(double meters) const;
  double hw_pose_units_to_radians(int hw_units) const;
  int radians_to_hw_pose_units(double radians) const;

  // Joint state and command storage
  std::vector<double> hw_joint_positions_;
  std::vector<double> hw_joint_velocities_;
  std::vector<double> hw_joint_position_commands_;

  // Gripper joint state and command storage (joint7 and joint8)
  std::vector<double> hw_gripper_positions_;
  std::vector<double> hw_gripper_velocities_;
  std::vector<double> hw_gripper_position_commands_;
  std::vector<double> hw_gripper_effort_commands_;

  // GPIO state and command storage for extended features
  // arm_admin GPIO
  double gpio_arm_enable_state_;
  double gpio_arm_enable_command_;
  double gpio_arm_connected_state_;

  // arm_current_pose GPIO
  double gpio_pose_x_state_;
  double gpio_pose_y_state_;
  double gpio_pose_z_state_;
  double gpio_pose_rx_state_;
  double gpio_pose_ry_state_;
  double gpio_pose_rz_state_;

  // arm_target_pose GPIO (Cartesian mode only)
  double gpio_target_pose_x_command_;
  double gpio_target_pose_y_command_;
  double gpio_target_pose_z_command_;
  double gpio_target_pose_rx_command_;
  double gpio_target_pose_ry_command_;
  double gpio_target_pose_rz_command_;

  // arm_status GPIO
  double gpio_ctrl_mode_state_;
  double gpio_arm_status_state_;
  double gpio_mode_feed_state_;
  double gpio_teach_status_state_;
  double gpio_motion_status_state_;
  double gpio_trajectory_num_state_;
  double gpio_err_code_comm_state_;
  double gpio_err_code_angle_state_;

  // arm_motion_mode GPIO
  double gpio_motion_mode_state_;
  double gpio_motion_mode_command_;
  double gpio_speed_state_;
  double gpio_speed_command_;

  // Hardware communication
  std::unique_ptr<agilex::piper::PiperController> piper_controller_;

  // Configuration parameters
  std::string can_interface_;
  bool include_gripper_;
  int motion_mode_;
  int speed_;

  // Runtime state
  bool hardware_connected_;
  bool first_read_completed_;

  // Constants
  static constexpr double HW_TO_RAD_FACTOR = M_PI / 180000.0;  // Convert 0.001deg to rad
  static constexpr double HW_TO_METER_FACTOR = 0.000001;       // Convert 0.001mm to meters
  static constexpr double HW_TO_NM_FACTOR = 0.001;             // Convert 0.001Nm to Nm
  static constexpr size_t NUM_JOINTS = 6;
  static constexpr size_t NUM_GRIPPER_JOINTS = 2;

  // Gripper control constants
  static constexpr uint16_t DEFAULT_GRIPPER_EFFORT = 1000;  // in 0.001 N⋅m
  static constexpr uint8_t GRIPPER_ENABLE = 0x01;
  static constexpr uint8_t GRIPPER_DISABLE_CLEAR = 0x02;
};

}  // namespace agilex_piper_ros2_control
