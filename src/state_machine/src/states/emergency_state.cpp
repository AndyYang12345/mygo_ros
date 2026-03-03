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

void EmergencyState::onEnter(RobotStateMachineNode *context)
{
	context->sendStopCommands();
	context->setMenuItems({"B恢复到上一个状态", "X回到IDLE"});
	context->setMenuSelection(0);
	RCLCPP_ERROR(context->get_logger(), "Entered EMERGENCY state");
}

void EmergencyState::handleButton(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
	if (msg->event_type != 0)
	{
		return;
	}

	if (msg->button_id == 1)
	{
		context->changeState(context->getStateBeforeEmergency());
	}
	else if (msg->button_id == 2)
	{
		context->changeState(1);
	}
	else if (msg->button_id == 8)
	{
		context->changeState(context->getStateBeforeEmergency());
	}
}

void EmergencyState::handleCombo(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ComboIntent::SharedPtr msg)
{
	if (msg->combo_name == "LT_RT_CONFIRM")
	{
		context->changeState(context->getStateBeforeEmergency());
	}
}

