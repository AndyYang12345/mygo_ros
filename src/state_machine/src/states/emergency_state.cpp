#include "state_machine/emergency_state.hpp"

#include "state_machine/robot_state_machine_node.hpp"

std::string EmergencyState::getName() const
{
  return "EMERGENCY";
}

uint8_t EmergencyState::getStateEnum() const
{
  return 6;
}

void EmergencyState::onEnter(RobotStateMachineNode * context)
{
  RCLCPP_WARN(context->get_logger(), "EMERGENCY STOP ACTIVATED!");
  context->sendStopCommands();
}

void EmergencyState::handleButton(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
  if (msg->button_id == 0 && msg->event_type == 0) {
    RCLCPP_INFO(context->get_logger(), "EMERGENCY: A pressed -> returning to IDLE");
    context->changeState(1);
  }
}

void EmergencyState::handleCombo(
  RobotStateMachineNode * context,
  const custom_interfaces::msg::ComboIntent::SharedPtr msg)
{
  if (msg->combo_name == "LT_RT") {
    RCLCPP_INFO(context->get_logger(), "EMERGENCY: LT+RT combo -> clearing emergency");
    context->changeState(1);
  }
}
