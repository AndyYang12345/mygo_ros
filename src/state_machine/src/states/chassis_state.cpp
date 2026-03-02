#include "state_machine/chassis_state.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "state_machine/robot_state_machine_node.hpp"

std::string ChassisState::getName() const
{
  return "CHASSIS";
}

uint8_t ChassisState::getStateEnum() const
{
  return 2;
}

void ChassisState::onEnter(RobotStateMachineNode * context)
{
  RCLCPP_INFO(context->get_logger(), "Entered CHASSIS mode");
  speed_multiplier_ = 1.0;
}

void ChassisState::onExit(RobotStateMachineNode * context)
{
  RCLCPP_INFO(context->get_logger(), "Exited CHASSIS mode");
  context->sendStopCommands();
}

void ChassisState::handleButton(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
  if (msg->event_type != 0) {
    return;
  }

  switch (msg->button_id) {
    case 2:
      RCLCPP_INFO(context->get_logger(), "CHASSIS: X pressed -> switching to ARM");
      context->changeState(3);
      break;
    case 3:
      RCLCPP_INFO(context->get_logger(), "CHASSIS: Y pressed -> switching to MENU");
      context->setMenuItems({"ARM Control", "Gripper", "Preset 1", "Preset 2", "Home"});
      context->setMenuSelection(0);
      context->changeState(4);
      break;
    case 4:
      speed_multiplier_ = std::max(0.3, speed_multiplier_ - 0.2);
      RCLCPP_INFO(context->get_logger(), "CHASSIS: Speed multiplier = %.2f", speed_multiplier_);
      break;
    case 5:
      speed_multiplier_ = std::min(2.0, speed_multiplier_ + 0.2);
      RCLCPP_INFO(context->get_logger(), "CHASSIS: Speed multiplier = %.2f", speed_multiplier_);
      break;
    default:
      break;
  }
}

void ChassisState::handleJoystick(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
  if (msg->joystick_id != 0) {
    return;
  }

  auto apply_deadzone = [context](float val) {
    if (std::abs(val) < context->getJoystickDeadzone()) {
      return 0.0f;
    }
    return val;
  };

  float x = apply_deadzone(msg->x);
  float y = apply_deadzone(msg->y);

  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = y * context->getChassisMaxLinearSpeed() * speed_multiplier_;
  twist.angular.z = x * context->getChassisMaxAngularSpeed() * speed_multiplier_;

  context->getChassisCmdPub()->publish(twist);
}

void ChassisState::handleTrigger(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
  (void)context;
  if (msg->trigger_id == 0) {
    speed_multiplier_ = 1.0 - msg->value * 0.5;
  } else if (msg->trigger_id == 1) {
    speed_multiplier_ = 1.0 + msg->value * 1.0;
  }
}

void ChassisState::update(RobotStateMachineNode * context)
{
  auto now = context->now();
  if ((now - context->getLeftJoystick().timestamp).seconds() > 0.2) {
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
  }
}
