#include "state_machine/chassis_state.hpp"
#include "state_machine/robot_state_machine_node.hpp"

std::string ChassisState::getName() const
{
	return "CHASSIS";
}

uint8_t ChassisState::getStateEnum() const
{
	return 2;
}

void ChassisState::onEnter(RobotStateMachineNode *context)
{
	(void)context;
}

void ChassisState::onExit(RobotStateMachineNode *context)
{
	(void)context;
}

void ChassisState::handleButton(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ChassisState::handleJoystick(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ChassisState::handleTrigger(
	RobotStateMachineNode *context,
	const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
	(void)context;
	(void)msg;
}

void ChassisState::update(RobotStateMachineNode *context)
{
	(void)context;
}

