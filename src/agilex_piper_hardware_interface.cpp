// ********************************************************************************************************************
// Copyright [2026] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
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
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Validate the number of joints
  if (info_.joints.size() < NUM_JOINTS) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), "Expected %zu joints, got %zu",
      NUM_JOINTS, info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Parse hardware parameters
  can_interface_ = info_.hardware_parameters.at("can_interface");
  if (can_interface_.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("AgilexPiperHardwareInterface"),
      "Parameter 'can_interface' is required but not specified");
    return CallbackReturn::ERROR;
  }

  // Parse include_gripper parameter
  include_gripper_ = true;
  auto it = info_.hardware_parameters.find("include_gripper");
  if (it != info_.hardware_parameters.end()) {
    include_gripper_ = (it->second == "true" || it->second == "True" || it->second == "1");
  }

  // Parse motion_mode parameter (default: 1 = joint mode)
  motion_mode_ = 1;
  it = info_.hardware_parameters.find("motion_mode");
  if (it != info_.hardware_parameters.end()) {
    try {
      motion_mode_ = std::stoi(it->second);
      if (motion_mode_ < 0 || motion_mode_ > 1) {
        RCLCPP_WARN(
          rclcpp::get_logger("AgilexPiperHardwareInterface"),
          "Invalid motion_mode value: %d. Using default value 1 (joint mode)", motion_mode_);
        motion_mode_ = 1;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to parse motion_mode parameter: %s. Using default value 1", e.what());
      motion_mode_ = 1;
    }
  }

  // Parse speed parameter (default: 50)
  speed_ = 50;
  it = info_.hardware_parameters.find("speed");
  if (it != info_.hardware_parameters.end()) {
    try {
      speed_ = std::stoi(it->second);
      if (speed_ < 1 || speed_ > 100) {
        RCLCPP_WARN(
          rclcpp::get_logger("AgilexPiperHardwareInterface"),
          "Invalid speed value: %d. Speed should be between 1-100. Using default value 50", speed_);
        speed_ = 50;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to parse speed parameter: %s. Using default value 50", e.what());
      speed_ = 50;
    }
  }

  // Initialize joint data structures
  hw_joint_positions_.resize(NUM_JOINTS, 0.0);
  hw_joint_velocities_.resize(NUM_JOINTS, 0.0);
  hw_joint_position_commands_.resize(NUM_JOINTS, 0.0);

  // Initialize gripper joint data structures
  if (include_gripper_) {
    hw_gripper_positions_.resize(NUM_GRIPPER_JOINTS, 0.0);
    hw_gripper_velocities_.resize(NUM_GRIPPER_JOINTS, 0.0);
    hw_gripper_position_commands_.resize(NUM_GRIPPER_JOINTS, 0.0);
    hw_gripper_effort_commands_.resize(NUM_GRIPPER_JOINTS, 0.0);
  }

  // Initialize GPIO state and command variables
  initialize_gpio_interfaces();

  // Initialize state
  hardware_connected_ = false;
  first_read_completed_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("AgilexPiperHardwareInterface"),
    "Initialized with CAN interface: %s, include_gripper: %s, motion_mode: %d, speed: %d",
    can_interface_.c_str(), include_gripper_ ? "true" : "false", motion_mode_, speed_);

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

  // Export gripper joint state interfaces (joint7 and joint8)
  if (include_gripper_) {
    // Look for joints beyond the arm joints (assuming they are gripper joints)
    for (size_t i = NUM_JOINTS; i < info_.joints.size() && i < NUM_JOINTS + NUM_GRIPPER_JOINTS;
         ++i) {
      size_t gripper_idx = i - NUM_JOINTS;
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &hw_gripper_positions_[gripper_idx]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &hw_gripper_velocities_[gripper_idx]));
    }
  }

  // Export GPIO state interfaces for extended features
  // arm_admin GPIO state interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_admin", "enable_arm", &gpio_arm_enable_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_admin", "connected", &gpio_arm_connected_state_));

  // arm_current_pose GPIO state interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_current_pose", "x", &gpio_pose_x_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_current_pose", "y", &gpio_pose_y_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_current_pose", "z", &gpio_pose_z_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_current_pose", "rx", &gpio_pose_rx_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_current_pose", "ry", &gpio_pose_ry_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_current_pose", "rz", &gpio_pose_rz_state_));

  // arm_status GPIO state interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_status", "ctrl_mode", &gpio_ctrl_mode_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_status", "arm_status", &gpio_arm_status_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_status", "mode_feed", &gpio_mode_feed_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_status", "teach_status", &gpio_teach_status_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_status", "motion_status", &gpio_motion_status_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "arm_status", "trajectory_num", &gpio_trajectory_num_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_status", "err_code_comm", &gpio_err_code_comm_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "arm_status", "err_code_angle", &gpio_err_code_angle_state_));

  // arm_motion_mode GPIO state interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_motion_mode", "mode", &gpio_motion_mode_state_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("arm_motion_mode", "speed", &gpio_speed_state_));

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

  // Export gripper joint command interfaces (joint7 and joint8)
  if (include_gripper_) {
    for (size_t i = NUM_JOINTS; i < info_.joints.size() && i < NUM_JOINTS + NUM_GRIPPER_JOINTS;
         ++i) {
      size_t gripper_idx = i - NUM_JOINTS;
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &hw_gripper_position_commands_[gripper_idx]));
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
          &hw_gripper_effort_commands_[gripper_idx]));
    }
  }

  // Export GPIO command interfaces for extended features
  // arm_admin GPIO command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_admin", "enable_arm", &gpio_arm_enable_command_));

  // arm_target_pose GPIO command interfaces (Cartesian mode only)
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_target_pose", "x", &gpio_target_pose_x_command_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_target_pose", "y", &gpio_target_pose_y_command_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_target_pose", "z", &gpio_target_pose_z_command_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_target_pose", "rx", &gpio_target_pose_rx_command_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_target_pose", "ry", &gpio_target_pose_ry_command_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_target_pose", "rz", &gpio_target_pose_rz_command_));

  // arm_motion_mode GPIO command interfaces
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_motion_mode", "mode", &gpio_motion_mode_command_));
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("arm_motion_mode", "speed", &gpio_speed_command_));

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
    } else {
      gpio_arm_enable_state_ = 1.0;
      gpio_arm_enable_command_ = 1.0;
    }

    // Enable gripper
    if (include_gripper_) {
      if (!piper_controller_->control_gripper(0, DEFAULT_GRIPPER_EFFORT, GRIPPER_ENABLE, 0x00)) {
        RCLCPP_WARN(rclcpp::get_logger("AgilexPiperHardwareInterface"), "Failed to enable gripper");
      }
    }

    // Set control mode to position control with configured motion mode and speed
    if (!piper_controller_->set_mode(0x01, motion_mode_, speed_)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to set control mode with motion_mode: %d, speed: %d", motion_mode_, speed_);
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

    // Initialize gripper commands to current positions
    if (include_gripper_) {
      std::copy(
        hw_gripper_positions_.begin(), hw_gripper_positions_.end(),
        hw_gripper_position_commands_.begin());

      // Initialize gripper effort commands to default values
      std::fill(
        hw_gripper_effort_commands_.begin(), hw_gripper_effort_commands_.end(),
        static_cast<double>(DEFAULT_GRIPPER_EFFORT) * HW_TO_NM_FACTOR);
    }

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
      // Disable arm motors and gripper safely
      piper_controller_->disable_arm();
      if (include_gripper_) {
        piper_controller_->control_gripper(0, DEFAULT_GRIPPER_EFFORT, GRIPPER_DISABLE_CLEAR, 0x00);
      }
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
    std::vector<double> prev_gripper_positions;
    if (include_gripper_) {
      prev_gripper_positions = hw_gripper_positions_;
    }

    // Read joint positions from hardware
    auto arm_joint = piper_controller_->get_arm_joint();

    // Convert hardware units to radians and update joint positions
    hw_joint_positions_[0] = hw_units_to_rad(arm_joint.j1);
    hw_joint_positions_[1] = hw_units_to_rad(arm_joint.j2);
    hw_joint_positions_[2] = hw_units_to_rad(arm_joint.j3);
    hw_joint_positions_[3] = hw_units_to_rad(arm_joint.j4);
    hw_joint_positions_[4] = hw_units_to_rad(arm_joint.j5);
    hw_joint_positions_[5] = hw_units_to_rad(arm_joint.j6);

    // Read gripper state from hardware and update gripper joint positions
    if (include_gripper_) {
      auto gripper_state = piper_controller_->get_arm_gripper();
      double api_gripper_position = hw_gripper_units_to_meters(gripper_state.grippers_angle);
      update_gripper_positions_from_api(api_gripper_position);
    }

    // Calculate joint velocities (finite difference)
    if (first_read_completed_ && period.seconds() > 0.0) {
      for (size_t i = 0; i < NUM_JOINTS; ++i) {
        hw_joint_velocities_[i] = (hw_joint_positions_[i] - prev_positions[i]) / period.seconds();
      }
      // Calculate gripper joint velocities
      if (include_gripper_) {
        for (size_t i = 0; i < NUM_GRIPPER_JOINTS; ++i) {
          hw_gripper_velocities_[i] =
            (hw_gripper_positions_[i] - prev_gripper_positions[i]) / period.seconds();
        }
      }
    } else {
      std::fill(hw_joint_velocities_.begin(), hw_joint_velocities_.end(), 0.0);
      if (include_gripper_) {
        std::fill(hw_gripper_velocities_.begin(), hw_gripper_velocities_.end(), 0.0);
      }
    }

    // Initialize commands to current positions on first read
    if (!first_read_completed_) {
      std::copy(
        hw_joint_positions_.begin(), hw_joint_positions_.end(),
        hw_joint_position_commands_.begin());
      if (include_gripper_) {
        std::copy(
          hw_gripper_positions_.begin(), hw_gripper_positions_.end(),
          hw_gripper_position_commands_.begin());
        // Initialize gripper effort commands to default values
        std::fill(
          hw_gripper_effort_commands_.begin(), hw_gripper_effort_commands_.end(),
          static_cast<double>(DEFAULT_GRIPPER_EFFORT) * HW_TO_NM_FACTOR);
      }
      first_read_completed_ = true;
    }

    // Update GPIO state interfaces
    update_gpio_states();

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
    // Apply GPIO commands first (may affect hardware state)
    apply_gpio_commands();

    // Send joint position commands only in Joint mode (mode 1)
    // Skip in Cartesian mode (mode 0) to avoid conflicts with built-in Cartesian control
    if (gpio_motion_mode_state_ != 0.0) {
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
    }

    // Handle gripper commands
    if (include_gripper_) {
      // JointGroupPositionController sends individual commands for both joints
      // joint7 (left finger): range 0 to +0.035m
      // joint8 (right finger): range -0.035m to 0
      // Hardware API expects total gripper opening width
      if (hw_gripper_position_commands_.size() >= 2) {
        // Calculate total gripper opening from individual joint positions
        // Since joint7 opens positively and joint8 opens negatively:
        // Total opening = |joint7_position| + |joint8_position|
        double total_opening =
          std::abs(hw_gripper_position_commands_[0]) + std::abs(hw_gripper_position_commands_[1]);

        // Clamp to valid range [0, 0.07m] (max opening is 2 * 0.035m)
        total_opening = std::clamp(total_opening, 0.0, 0.07);

        int gripper_position_cmd = meters_to_hw_gripper_units(total_opening);

        // Use the maximum effort command from either joint
        double effort_nm = std::max(
          std::abs(hw_gripper_effort_commands_[0]), std::abs(hw_gripper_effort_commands_[1]));

        // Clamp effort to valid range and use default if too small
        if (effort_nm < 0.1) {  // If effort command is too small, use default
          effort_nm = static_cast<double>(DEFAULT_GRIPPER_EFFORT) * HW_TO_NM_FACTOR;
        }

        uint16_t gripper_effort_cmd = nm_to_hw_gripper_effort_units(effort_nm);

        // Send gripper command (enable gripper with position and effort)
        if (!piper_controller_->control_gripper(
              gripper_position_cmd, gripper_effort_cmd, GRIPPER_ENABLE, 0x00)) {
          RCLCPP_ERROR(
            rclcpp::get_logger("AgilexPiperHardwareInterface"),
            "Failed to send gripper commands to hardware");
          return hardware_interface::return_type::ERROR;
        }
      }
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

double AgilexPiperHardwareInterface::hw_gripper_units_to_meters(int hw_units) const
{
  return static_cast<double>(hw_units) * HW_TO_METER_FACTOR;
}

int AgilexPiperHardwareInterface::meters_to_hw_gripper_units(double meters) const
{
  return static_cast<int>(meters / HW_TO_METER_FACTOR);
}

double AgilexPiperHardwareInterface::hw_gripper_effort_units_to_nm(uint16_t hw_units) const
{
  return static_cast<double>(hw_units) * HW_TO_NM_FACTOR;
}

uint16_t AgilexPiperHardwareInterface::nm_to_hw_gripper_effort_units(double nm) const
{
  return static_cast<uint16_t>(std::clamp(nm / HW_TO_NM_FACTOR, 0.0, 5000.0));
}

void AgilexPiperHardwareInterface::update_gripper_positions_from_api(double api_position)
{
  // The API position represents the total opening distance between the gripper fingers
  // Convert this to individual joint positions:
  // joint7 (left finger): moves from 0 (closed) to +half_opening (open)
  // joint8 (right finger): moves from 0 (closed) to -half_opening (open)

  double half_opening = api_position * 0.5;

  if (hw_gripper_positions_.size() >= 2) {
    hw_gripper_positions_[0] = half_opening;   // joint7: left finger position (positive)
    hw_gripper_positions_[1] = -half_opening;  // joint8: right finger position (negative)
  }
}

void AgilexPiperHardwareInterface::initialize_gpio_interfaces()
{
  // Initialize arm_admin GPIO
  gpio_arm_enable_state_ = 0.0;
  gpio_arm_enable_command_ = 0.0;
  gpio_arm_connected_state_ = 0.0;

  // Initialize arm_current_pose GPIO
  gpio_pose_x_state_ = 0.0;
  gpio_pose_y_state_ = 0.0;
  gpio_pose_z_state_ = 0.0;
  gpio_pose_rx_state_ = 0.0;
  gpio_pose_ry_state_ = 0.0;
  gpio_pose_rz_state_ = 0.0;

  // Initialize arm_target_pose GPIO
  gpio_target_pose_x_command_ = 0.0;
  gpio_target_pose_y_command_ = 0.0;
  gpio_target_pose_z_command_ = 0.0;
  gpio_target_pose_rx_command_ = 0.0;
  gpio_target_pose_ry_command_ = 0.0;
  gpio_target_pose_rz_command_ = 0.0;

  // Initialize arm_status GPIO
  gpio_ctrl_mode_state_ = 0.0;
  gpio_arm_status_state_ = 0.0;
  gpio_mode_feed_state_ = 0.0;
  gpio_teach_status_state_ = 0.0;
  gpio_motion_status_state_ = 0.0;
  gpio_trajectory_num_state_ = 0.0;
  gpio_err_code_comm_state_ = 0.0;
  gpio_err_code_angle_state_ = 0.0;

  // Initialize arm_motion_mode GPIO with configured values
  gpio_motion_mode_state_ = static_cast<double>(motion_mode_);
  gpio_motion_mode_command_ = static_cast<double>(motion_mode_);
  gpio_speed_state_ = static_cast<double>(speed_);
  gpio_speed_command_ = static_cast<double>(speed_);
}

void AgilexPiperHardwareInterface::update_gpio_states()
{
  if (!hardware_connected_ || !piper_controller_) {
    gpio_arm_connected_state_ = 0.0;
    return;
  }

  // Update arm_admin states
  gpio_arm_connected_state_ = piper_controller_->is_connected() ? 1.0 : 0.0;

  // Update arm_current_pose states from hardware
  try {
    auto end_pose = piper_controller_->get_arm_end_pose();
    gpio_pose_x_state_ = hw_pose_units_to_meters(end_pose.x);
    gpio_pose_y_state_ = hw_pose_units_to_meters(end_pose.y);
    gpio_pose_z_state_ = hw_pose_units_to_meters(end_pose.z);
    gpio_pose_rx_state_ = hw_pose_units_to_radians(end_pose.rx);
    gpio_pose_ry_state_ = hw_pose_units_to_radians(end_pose.ry);
    gpio_pose_rz_state_ = hw_pose_units_to_radians(end_pose.rz);
  } catch (const std::exception & e) {
    static auto clock = rclcpp::Clock();
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), clock, 5000,
      "Failed to update end pose states: %s", e.what());
  }

  // Update arm_status states from hardware
  try {
    auto arm_status = piper_controller_->get_arm_status();
    gpio_ctrl_mode_state_ = static_cast<double>(arm_status.ctrl_mode);
    gpio_arm_status_state_ = static_cast<double>(arm_status.arm_status);
    gpio_mode_feed_state_ = static_cast<double>(arm_status.mode_feed);
    gpio_teach_status_state_ = static_cast<double>(arm_status.teach_status);
    gpio_motion_status_state_ = static_cast<double>(arm_status.motion_status);
    gpio_trajectory_num_state_ = static_cast<double>(arm_status.trajectory_num);
    gpio_err_code_comm_state_ = static_cast<double>(arm_status.err_code_comm);
    gpio_err_code_angle_state_ = static_cast<double>(arm_status.err_code_angle);
  } catch (const std::exception & e) {
    static auto clock = rclcpp::Clock();
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("AgilexPiperHardwareInterface"), clock, 5000,
      "Failed to update arm status states: %s", e.what());
  }
}

void AgilexPiperHardwareInterface::apply_gpio_commands()
{
  if (!hardware_connected_ || !piper_controller_) {
    return;
  }

  // Apply arm_admin commands
  if (std::abs(gpio_arm_enable_command_ - gpio_arm_enable_state_) > 0.5) {
    bool enable_arm = gpio_arm_enable_command_ > 0.5;
    try {
      if (enable_arm) {
        if (piper_controller_->enable_arm()) {
          gpio_arm_enable_state_ = 1.0;
          piper_controller_->set_mode(0x01, motion_mode_, speed_);
          RCLCPP_INFO(
            rclcpp::get_logger("AgilexPiperHardwareInterface"), "Arm enabled via GPIO command");
        }
      } else {
        if (piper_controller_->disable_arm()) {
          gpio_arm_enable_state_ = 0.0;
          piper_controller_->set_mode(0x01, motion_mode_, speed_);
          RCLCPP_INFO(
            rclcpp::get_logger("AgilexPiperHardwareInterface"), "Arm disabled via GPIO command");
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to apply arm enable command: %s", e.what());
    }
  }

  // Apply arm_motion_mode commands
  if (
    std::abs(gpio_motion_mode_command_ - gpio_motion_mode_state_) > 0.5 ||
    std::abs(gpio_speed_command_ - gpio_speed_state_) > 0.5) {
    try {
      motion_mode_ = static_cast<int>(gpio_motion_mode_command_);
      speed_ = static_cast<int>(std::clamp(gpio_speed_command_, 1.0, 100.0));

      if (piper_controller_->set_mode(0x01, motion_mode_, speed_)) {
        gpio_motion_mode_state_ = gpio_motion_mode_command_;
        gpio_speed_state_ = gpio_speed_command_;
        RCLCPP_INFO(
          rclcpp::get_logger("AgilexPiperHardwareInterface"),
          "Motion mode set to %s with speed %d%% via GPIO command",
          motion_mode_ == 0 ? "Cartesian" : "Joint", speed_);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to apply motion mode command: %s", e.what());
    }
  }

  // Apply arm_target_pose commands (only valid in Cartesian motion mode)
  if (gpio_motion_mode_state_ == 0.0) {
    try {
      int x = meters_to_hw_pose_units(gpio_target_pose_x_command_);
      int y = meters_to_hw_pose_units(gpio_target_pose_y_command_);
      int z = meters_to_hw_pose_units(gpio_target_pose_z_command_);
      int rx = radians_to_hw_pose_units(gpio_target_pose_rx_command_);
      int ry = radians_to_hw_pose_units(gpio_target_pose_ry_command_);
      int rz = radians_to_hw_pose_units(gpio_target_pose_rz_command_);

      if (!piper_controller_->set_end_pose(x, y, z, rx, ry, rz)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("AgilexPiperHardwareInterface"),
          "Failed to send target pose command to hardware");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AgilexPiperHardwareInterface"),
        "Failed to apply target pose command: %s", e.what());
    }
  }
}

double AgilexPiperHardwareInterface::hw_pose_units_to_meters(int hw_units) const
{
  // Convert from hardware units (0.001 mm) to meters
  return static_cast<double>(hw_units) * 0.000001;
}

int AgilexPiperHardwareInterface::meters_to_hw_pose_units(double meters) const
{
  // Convert from meters to hardware units (0.001 mm)
  return static_cast<int>(meters * 1000000.0);
}

double AgilexPiperHardwareInterface::hw_pose_units_to_radians(int hw_units) const
{
  // Convert from hardware units (degrees * 1000) to radians
  return static_cast<double>(hw_units) * 0.001 * M_PI / 180.0;
}

int AgilexPiperHardwareInterface::radians_to_hw_pose_units(double radians) const
{
  // Convert from radians to hardware units (degrees * 1000)
  return static_cast<int>(radians * 180.0 / M_PI * 1000.0);
}

}  // namespace agilex_piper_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  agilex_piper_ros2_control::AgilexPiperHardwareInterface, hardware_interface::SystemInterface)
