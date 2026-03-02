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
	(void)context;
}

void EmergencyState::handleButton(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void EmergencyState::handleCombo(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ComboIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

