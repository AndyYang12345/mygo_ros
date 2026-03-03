#include "state_machine/arm_state.hpp"

#include <cmath>

#include "state_machine/robot_state_machine_node.hpp"

namespace {
double applyDeadzone(double value, double deadzone)
{
    return std::abs(value) < deadzone ? 0.0 : value;
}
}

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
    context->setMenuItems({"B返回菜单"});
    context->setMenuSelection(0);
    RCLCPP_INFO(context->get_logger(), "Entered ARM state");
}

void ArmState::onExit(RobotStateMachineNode *context)
{
    context->getArmCmdPub()->publish(geometry_msgs::msg::Twist());
}

void ArmState::handleButton(
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

void ArmState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    auto arm_twist = geometry_msgs::msg::Twist();

    if (msg->joystick_id == 0)
    {
        arm_twist.linear.x = applyDeadzone(msg->x, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        arm_twist.linear.y = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
    }
    else if (msg->joystick_id == 1)
    {
        arm_twist.linear.z = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        arm_twist.angular.z = applyDeadzone(msg->x, context->getJoystickDeadzone()) * context->getArmSpeedScale();
    }
    else if (msg->joystick_id == 2)
    {
        arm_twist.angular.y = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
    }
    else
    {
        return;
    }

    context->getArmCmdPub()->publish(arm_twist);
}

void ArmState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    auto gripper_cmd = std_msgs::msg::String();
    if (msg->trigger_id == 0)
    {
        gripper_cmd.data = "OPEN:" + std::to_string(-msg->value);
        context->getGripperCmdPub()->publish(gripper_cmd);
    }
    else if (msg->trigger_id == 1)
    {
        gripper_cmd.data = "CLOSE:" + std::to_string(-msg->value);
        context->getGripperCmdPub()->publish(gripper_cmd);
    }
}

void ArmState::handleCombo(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ComboIntent::SharedPtr msg)
{
    if (msg->combo_name != "LT_RT_CONFIRM")
    {
        return;
    }

    auto preset = std_msgs::msg::Int32();
    preset.data = 1;
    context->getPresetPub()->publish(preset);
}

void ArmState::update(RobotStateMachineNode *context)
{
    (void)context;
}
