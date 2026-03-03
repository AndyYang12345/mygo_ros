#include "state_machine/pole_state.hpp"

#include <cmath>

#include "state_machine/robot_state_machine_node.hpp"

std::string PoleState::getName() const
{
    return "POLE";
}

uint8_t PoleState::getStateEnum() const
{
    return 5;
}

void PoleState::onEnter(RobotStateMachineNode *context)
{
    pole_locked_ = false;
    context->setMenuItems({"退出杆子模式"});
    context->setMenuSelection(0);
    RCLCPP_INFO(context->get_logger(), "Entered POLE state");
}

void PoleState::onExit(RobotStateMachineNode *context)
{
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
    auto stop_cmd = std_msgs::msg::String();
    stop_cmd.data = "POLE_STOP";
    context->getGripperCmdPub()->publish(stop_cmd);
}

void PoleState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0)
    {
        return;
    }

    if (msg->button_id == 1)
    {
        context->changeState(4);
    }
}

void PoleState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id != 0)
    {
        return;
    }

    const auto x = applyDeadzone(msg->x, context->getJoystickDeadzone());
    const auto y = applyDeadzone(msg->y, context->getJoystickDeadzone());

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = y * context->getChassisMaxLinearSpeed();
    twist.angular.z = -x * context->getChassisMaxAngularSpeed();
    context->getChassisCmdPub()->publish(twist);
}

void PoleState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    if (msg->trigger_id == 1)
    {
        if (pole_locked_)
        {
            return;
        }
        auto pole_cmd = std_msgs::msg::String();
        pole_cmd.data = "POLE_ROTATE:" + std::to_string(msg->value);
        context->getGripperCmdPub()->publish(pole_cmd);
        return;
    }

    if (msg->trigger_id == 0 && msg->value < -0.8F)
    {
        pole_locked_ = true;
        auto pole_cmd = std_msgs::msg::String();
        pole_cmd.data = "POLE_LOCK";
        context->getGripperCmdPub()->publish(pole_cmd);
    }
}

void PoleState::update(RobotStateMachineNode *context)
{
    (void)context;
}

double PoleState::applyDeadzone(double value, double deadzone) const
{
    return std::abs(value) < deadzone ? 0.0 : value;
}
