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
  int homePresetIndex() const;
  void publishJointCommand(RobotStateMachineNode * context);
  void publishDirectPwmCommand(RobotStateMachineNode * context);
  bool applyAxisControl(RobotStateMachineNode * context, double dt);
  void savePoseSnapshot(RobotStateMachineNode * context);

  // 8-direction submenu slots; UI is expected to render these in octant order:
  // 0 RIGHT, 1 UP_RIGHT, 2 UP, 3 UP_LEFT, 4 LEFT, 5 DOWN_LEFT, 6 DOWN, 7 DOWN_RIGHT.
  // Use "-" as an explicit empty slot that should not trigger any motion.
  std::vector<std::string> presets_ = {
    "Right Energy Unit",  // RIGHT
    "under_bridge",       // UP_RIGHT
    "home",               // UP (top)
    "folded",             // UP_LEFT
    "Left Energy Unit",   // LEFT
    "-",                  // DOWN_LEFT
    "Side Energy Unit",   // DOWN
    "-"                   // DOWN_RIGHT
  };

  bool submenu_active_ = false;
  int submenu_selection_ = 0;

  bool precision_mode_ = false;
  bool gripper_open_ = true;
  bool rt_press_latched_ = false;
  bool dpad_switch_latched_ = false;
  int right_y_selected_servo_ = 2;

  std::array<double, 5> target_joints_rad_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 5> target_pwms_ = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0};
  bool target_joints_initialized_ = false;
  bool waiting_initial_state_ = true;
  bool preset_sync_pending_ = false;
  bool preset_motion_in_progress_ = false;
  bool preset_feedback_gate_ = false;
  bool preset_feedback_query_sent_ = false;
  bool kg_sync_requested_ = false;
  bool latest_feedback_valid_ = false;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_query_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time preset_sync_due_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_sync_query_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr current_joint_sub_;
  std::array<double, 5> latest_feedback_joints_rad_ = {0.0, 0.0, 0.0, 0.0, 0.0};

  std::array<double, 5> max_speed_high_pwm_s_ = {260.0, 220.0, 180.0, 180.0, 220.0};
  std::array<double, 5> max_speed_precision_pwm_s_ = {85.0, 70.0, 55.0, 55.0, 70.0};
  double preset_sync_delay_s_ = 1.2;
  double kg_sync_interval_s_ = 2.0;
  double kg_query_timeout_s_ = 0.8;
  double min_step_pwm_ = 0.6;

  const std::array<double, 5> min_pwm_ = {500.0, 500.0, 500.0, 500.0, 500.0};
  const std::array<double, 5> max_pwm_ = {2500.0, 2500.0, 2500.0, 2500.0, 2500.0};
};
