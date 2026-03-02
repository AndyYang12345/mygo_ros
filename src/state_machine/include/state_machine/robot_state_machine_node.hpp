#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "custom_interfaces/msg/button_intent.hpp"
#include "custom_interfaces/msg/combo_intent.hpp"
#include "custom_interfaces/msg/joystick_intent.hpp"
#include "custom_interfaces/msg/robot_state.hpp"
#include "custom_interfaces/msg/trigger_intent.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine/robot_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class RobotStateMachineNode : public rclcpp::Node {
public:
  RobotStateMachineNode();

  void changeState(uint8_t state_enum);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr getChassisCmdPub();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr getArmCmdPub();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr getGripperCmdPub();
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr getPresetPub();
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr getJointTrajectoryPub();

  double getChassisMaxLinearSpeed() const;
  double getChassisMaxAngularSpeed() const;
  double getArmSpeedScale() const;
  double getJoystickDeadzone() const;

  struct JoystickData {
    float x = 0.0;
    float y = 0.0;
    rclcpp::Time timestamp;
  };

  const JoystickData & getLeftJoystick() const;
  const JoystickData & getRightJoystick() const;

  void setMenuItems(const std::vector<std::string> & items);
  const std::vector<std::string> & getMenuItems() const;
  void setMenuSelection(int index);
  int getMenuSelection() const;

  void sendStopCommands();

private:
  void declareParameters();
  void setupPublishers();
  void setupSubscribers();
  void setupTimers();

  bool isTransitionAllowed(uint8_t from, uint8_t to);
  void publishState();

  std::map<uint8_t, std::shared_ptr<RobotState>> states_;
  RobotState * current_state_ = nullptr;

  struct Parameters {
    double chassis_max_linear_speed = 0.5;
    double chassis_max_angular_speed = 1.0;
    double arm_speed_scale = 0.05;
    double joystick_deadzone = 0.1;
    int control_frequency = 50;
  } params_;

  std::vector<std::string> menu_items_;
  int menu_selection_ = 0;

  JoystickData left_joystick_;
  JoystickData right_joystick_;

  rclcpp::Publisher<custom_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr preset_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;

  rclcpp::Subscription<custom_interfaces::msg::ButtonIntent>::SharedPtr button_sub_;
  rclcpp::Subscription<custom_interfaces::msg::JoystickIntent>::SharedPtr joystick_sub_;
  rclcpp::Subscription<custom_interfaces::msg::TriggerIntent>::SharedPtr trigger_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ComboIntent>::SharedPtr combo_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};
