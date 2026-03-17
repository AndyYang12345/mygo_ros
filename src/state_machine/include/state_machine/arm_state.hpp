#pragma once

#include <array>
#include <string>
#include <vector>

#include "example_interfaces/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine/robot_state.hpp"

class ArmState : public RobotState {
public:
  std::string getName() const override;
  uint8_t getStateEnum() const override;

  void onEnter(RobotStateMachineNode * context) override;
  void onExit(RobotStateMachineNode * context) override;
  void handleButton(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg) override;
  void handleJoystick(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg) override;
  void handleTrigger(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg) override;
  void handleCombo(
    RobotStateMachineNode * context,
    const custom_interfaces::msg::ComboIntent::SharedPtr msg) override;
  void update(RobotStateMachineNode * context) override;

  uint8_t getSubState() const override;
  std::vector<std::string> getAvailableModes() const override;

private:
  void updateSubmenuUi(RobotStateMachineNode * context);
  int angleToOctant(float x, float y) const;
  void publishJointCommand(RobotStateMachineNode * context);
  void applyAxisControl(RobotStateMachineNode * context, double dt);

  std::vector<std::string> presets_ = {
    "pickup_right",
    "pose_1",
    "home",
    "pose_2",
    "pickup_left",
    "box_left",
    "normal_detection",
    "box_right"
  };

  bool submenu_active_ = false;
  int submenu_selection_ = 0;

  bool precision_mode_ = false;
  bool gripper_open_ = true;
  bool rt_press_latched_ = false;
  bool dpad_switch_latched_ = false;
  int right_y_selected_servo_ = 2;

  std::array<double, 5> target_joints_rad_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  bool target_joints_initialized_ = false;
  bool waiting_initial_state_ = true;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_query_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr current_joint_sub_;

  std::array<double, 5> max_speed_high_rad_s_ = {1.20, 1.00, 0.90, 0.90, 1.20};
  std::array<double, 5> max_speed_precision_rad_s_ = {0.35, 0.30, 0.25, 0.25, 0.35};

  const std::array<double, 5> min_joint_rad_ = {
    -2.35619449019,
    -2.35619449019,
    -2.35619449019,
    -2.35619449019,
    -2.35619449019};
  const std::array<double, 5> max_joint_rad_ = {
    2.35619449019,
    2.35619449019,
    2.35619449019,
    2.35619449019,
    2.35619449019};
};
