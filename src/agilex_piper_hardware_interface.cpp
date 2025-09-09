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
#include "agilex_piper_ros2_control/agilex_piper_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agilex_piper_ros2_control
{

hardware_interface::CallbackReturn AgilexPiperHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Validate the number of joints
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), "Expected %zu joints, got %zu",
      NUM_JOINTS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Initialize joint data structures
  hw_joint_positions_.resize(NUM_JOINTS, 0.0);
  hw_joint_velocities_.resize(NUM_JOINTS, 0.0);
  hw_joint_position_commands_.resize(NUM_JOINTS, 0.0);

  // Initialize state
  hardware_connected_ = false;
  first_read_completed_ = false;

  // Parse hardware parameters
  can_interface_ = info_.hardware_parameters.at("can_interface");
  if (can_interface_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"),
      "Parameter 'can_interface' is required but not specified");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AgilexPiperHardwareInterface"), "Initialized with CAN interface: %s",
    can_interface_.c_str());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AgilexPiperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export joint state interfaces
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_positions_[i]));

    // Velocity states - needed by cartesian_motion_controller
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_joint_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AgilexPiperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export joint command interfaces
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_joint_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AgilexPiperHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AgilexPiperHardwareInterface"), "Activating hardware interface...");

  try {
    // Initialize Piper controller
    piper_controller_ =
      std::make_unique<agilex::piper::PiperController>(can_interface_, true, 0, false);

    // Check connection
    hardware_connected_ = piper_controller_->is_connected();
    if (!hardware_connected_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to connect to Piper arm on CAN interface: %s", can_interface_.c_str());
      return CallbackReturn::ERROR;
    }

    // Enable arm motors
    if (!piper_controller_->enable_arm()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"), "Failed to enable arm motors");
      return CallbackReturn::ERROR;
    }

    // Set control mode to position control
    if (!piper_controller_->set_mode(0x01, 0x01, 50)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to set control mode to position mode");
      return CallbackReturn::ERROR;
    }

    // Read initial state
    if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"), "Failed to read initial joint states");
      return CallbackReturn::ERROR;
    }

    // Initialize commands to current positions
    std::copy(
      hw_joint_positions_.begin(), hw_joint_positions_.end(), hw_joint_position_commands_.begin());

    RCLCPP_INFO(
      rclcpp::get_logger("AgilexPiperHardwareInterface"),
      "Hardware interface activated successfully");

    return CallbackReturn::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), "Exception during activation: %s",
      e.what());
    return CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn AgilexPiperHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AgilexPiperHardwareInterface"), "Deactivating hardware interface...");

  try {
    if (hardware_connected_ && piper_controller_) {
      // Disable arm motors safely
      piper_controller_->disable_arm();
      piper_controller_->disconnect();
      piper_controller_.reset();
    }

    hardware_connected_ = false;
    first_read_completed_ = false;

    RCLCPP_INFO(
      rclcpp::get_logger("AgilexPiperHardwareInterface"),
      "Hardware interface deactivated successfully");

    return CallbackReturn::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), "Exception during deactivation: %s",
      e.what());
    return CallbackReturn::ERROR;
  }
}

hardware_interface::return_type AgilexPiperHardwareInterface::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  if (!hardware_connected_ || !piper_controller_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Store previous positions for velocity calculation
    std::vector<double> prev_positions = hw_joint_positions_;

    // Read joint positions from hardware
    auto arm_joint = piper_controller_->get_arm_joint();

    // Convert hardware units to radians and update joint positions
    hw_joint_positions_[0] = hw_units_to_rad(arm_joint.j1);
    hw_joint_positions_[1] = hw_units_to_rad(arm_joint.j2);
    hw_joint_positions_[2] = hw_units_to_rad(arm_joint.j3);
    hw_joint_positions_[3] = hw_units_to_rad(arm_joint.j4);
    hw_joint_positions_[4] = hw_units_to_rad(arm_joint.j5);
    hw_joint_positions_[5] = hw_units_to_rad(arm_joint.j6);

    // Calculate joint velocities (finite difference)
    if (first_read_completed_ && period.seconds() > 0.0) {
      for (size_t i = 0; i < NUM_JOINTS; ++i) {
        hw_joint_velocities_[i] = (hw_joint_positions_[i] - prev_positions[i]) / period.seconds();
      }
    } else {
      std::fill(hw_joint_velocities_.begin(), hw_joint_velocities_.end(), 0.0);
    }

    // Initialize commands to current positions on first read
    if (!first_read_completed_) {
      std::copy(
        hw_joint_positions_.begin(), hw_joint_positions_.end(),
        hw_joint_position_commands_.begin());
      first_read_completed_ = true;
    }

    return hardware_interface::return_type::OK;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), "Exception during read: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type AgilexPiperHardwareInterface::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!hardware_connected_ || !piper_controller_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Convert joint commands from radians to hardware units
    int j1_cmd = rad_to_hw_units(hw_joint_position_commands_[0]);
    int j2_cmd = rad_to_hw_units(hw_joint_position_commands_[1]);
    int j3_cmd = rad_to_hw_units(hw_joint_position_commands_[2]);
    int j4_cmd = rad_to_hw_units(hw_joint_position_commands_[3]);
    int j5_cmd = rad_to_hw_units(hw_joint_position_commands_[4]);
    int j6_cmd = rad_to_hw_units(hw_joint_position_commands_[5]);

    // Send position commands to hardware
    if (!piper_controller_->set_joint_angles(j1_cmd, j2_cmd, j3_cmd, j4_cmd, j5_cmd, j6_cmd)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to send joint position commands to hardware");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), "Exception during write: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

double AgilexPiperHardwareInterface::hw_units_to_rad(int hw_units) const
{
  return static_cast<double>(hw_units) * HW_TO_RAD_FACTOR;
}

int AgilexPiperHardwareInterface::rad_to_hw_units(double rad) const
{
  return static_cast<int>(rad / HW_TO_RAD_FACTOR);
}

}  // namespace agilex_piper_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  agilex_piper_ros2_control::AgilexPiperHardwareInterface, hardware_interface::SystemInterface)
