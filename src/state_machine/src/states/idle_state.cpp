#include "state_machine/idle_state.hpp"

#include "state_machine/robot_state_machine_node.hpp"

std::string IdleState::getName() const
{
  return "IDLE";
}

uint8_t IdleState::getStateEnum() const
{
  return 1;
}

void IdleState::onEnter(RobotStateMachineNode * context)
{
  RCLCPP_INFO(context->get_logger(), "Entered IDLE state");
}

void IdleState::handleButton(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
  if (msg->event_type != 0) {
    return;
  }

  switch (msg->button_id) {
    case 0:
      RCLCPP_INFO(context->get_logger(), "IDLE: A pressed -> switching to CHASSIS");
      context->changeState(2);
      break;
    case 1:
      RCLCPP_INFO(context->get_logger(), "IDLE: B pressed -> switching to ARM");
      context->changeState(3);
      break;
    case 2:
      RCLCPP_INFO(context->get_logger(), "IDLE: X pressed -> switching to MENU");
      context->changeState(4);
      break;
    default:
      break;
  }
}
