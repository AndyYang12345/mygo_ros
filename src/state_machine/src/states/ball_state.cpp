#include "state_machine/ball_state.hpp"

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "state_machine/robot_state_machine_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
double applyDeadzone(double value, double deadzone)
{
    return std::abs(value) < deadzone ? 0.0 : value;
}
}

std::string BallState::getName() const
{
    return "BALL";
}

uint8_t BallState::getStateEnum() const
{
    return 6;
}

void BallState::onEnter(RobotStateMachineNode *context)
{
    a_pressed_latched_ = false;
    context->setMenuItems({"A:发送下一条小球提交指令", "B:返回上一个状态", "X:回到IDLE"});
    context->setMenuSelection(0);
    RCLCPP_INFO(context->get_logger(), "Entered BALL state");
}

void BallState::onExit(RobotStateMachineNode *context)
{
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
}

void BallState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    if (msg->button_id == 0)
    {
        if (msg->event_type == 0 && !a_pressed_latched_)
        {
            a_pressed_latched_ = true;
            auto trigger = std_msgs::msg::String();
            trigger.data = "NEXT";
            context->getCollectorCmdPub()->publish(trigger);
            RCLCPP_INFO(context->get_logger(), "BALL: A pressed -> trigger collector next command");
        }
        else if (msg->event_type == 1)
        {
            a_pressed_latched_ = false;
        }
        return;
    }

    if (msg->event_type != 0)
    {
        return;
    }
}

void BallState::update(RobotStateMachineNode *context)
{
    const auto &joystick = context->getLeftJoystick();
    if (!joystick.is_active(context))
    {
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        return;
    }

    const auto forward = applyDeadzone(joystick.y, context->getJoystickDeadzone());
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = forward * context->getChassisMaxLinearSpeed();
    twist.angular.z = 0.0;
    context->getChassisCmdPub()->publish(twist);
}
