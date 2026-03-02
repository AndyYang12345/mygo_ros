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

void IdleState::onEnter(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "Entered IDLE state");
}