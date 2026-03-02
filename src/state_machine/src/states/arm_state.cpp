#include "state_machine/arm_state.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "state_machine/robot_state_machine_node.hpp"
#include "std_msgs/msg/string.hpp"

std::string ArmState::getName() const
{
  return "ARM";
}

uint8_t ArmState::getStateEnum() const
{
  return 3;
}

void ArmState::onEnter(RobotStateMachineNode * context)
{
  RCLCPP_INFO(context->get_logger(), "Entered ARM mode");
}

void ArmState::onExit(RobotStateMachineNode * context)
{
  RCLCPP_INFO(context->get_logger(), "Exited ARM mode");
  context->getArmCmdPub()->publish(geometry_msgs::msg::Twist());
}

void ArmState::handleButton(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
  if (msg->event_type != 0) {
    return;
  }

  switch (msg->button_id) {
    case 0:
    {
      auto gripper_cmd = std_msgs::msg::String();
      gripper_cmd.data = "CLOSE";
      context->getGripperCmdPub()->publish(gripper_cmd);
      RCLCPP_INFO(context->get_logger(), "ARM: Gripper close");
      break;
    }
    case 1:
    {
      auto gripper_cmd = std_msgs::msg::String();
      gripper_cmd.data = "OPEN";
      context->getGripperCmdPub()->publish(gripper_cmd);
      RCLCPP_INFO(context->get_logger(), "ARM: Gripper open");
      break;
    }
    case 2:
      RCLCPP_INFO(context->get_logger(), "ARM: Go to home");
      break;
    case 3:
      RCLCPP_INFO(context->get_logger(), "ARM: Y pressed -> switching to CHASSIS");
      context->changeState(2);
      break;
    default:
      break;
  }
}

void ArmState::handleJoystick(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
  auto arm_twist = geometry_msgs::msg::Twist();
  double scale = context->getArmSpeedScale();

  if (msg->joystick_id == 0) {
    arm_twist.linear.x = msg->x * scale;
    arm_twist.linear.y = msg->y * scale;
  } else if (msg->joystick_id == 1) {
    arm_twist.linear.z = msg->y * scale;
    arm_twist.angular.z = msg->x * scale;
  }

  context->getArmCmdPub()->publish(arm_twist);
}

void ArmState::handleTrigger(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
  (void)context;
  (void)msg;
}

void ArmState::update(RobotStateMachineNode * context)
{
  (void)context;
}
