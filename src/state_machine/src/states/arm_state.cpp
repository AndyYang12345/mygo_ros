#include "state_machine/arm_state.hpp"
#include "state_machine/robot_state_machine_node.hpp"

std::string ArmState::getName() const
{
	return "ARM";
}

uint8_t ArmState::getStateEnum() const
{
	return 3;
}

void ArmState::onEnter(RobotStateMachineNode *context)
{
	(void)context;
}

void ArmState::onExit(RobotStateMachineNode *context)
{
	(void)context;
}

void ArmState::handleButton(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ArmState::handleJoystick(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ArmState::handleTrigger(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ArmState::update(RobotStateMachineNode *context)
{
	(void)context;
}

