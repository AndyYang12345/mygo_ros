#include "state_machine/vision_task_state.hpp"

#include <cmath>

#include "state_machine/robot_state_machine_node.hpp"

std::string VisionTaskState::getName() const
{
    return "VISION_TASK";
}

uint8_t VisionTaskState::getStateEnum() const
{
    return 7;
}

uint8_t VisionTaskState::getSubState() const
{
    return task_active_ ? 1 : 0;
}

void VisionTaskState::onEnter(RobotStateMachineNode *context)
{
    task_active_ = false;
    context->setMenuItems({"A开始视觉任务", "B取消并返回菜单"});
    context->setMenuSelection(0);
    RCLCPP_INFO(context->get_logger(), "Entered VISION_TASK state");
}

void VisionTaskState::onExit(RobotStateMachineNode *context)
{
    context->sendStopCommands();
}

void VisionTaskState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0)
    {
        return;
    }

    if (msg->button_id == 0)
    {
        task_active_ = true;
        auto preset = std_msgs::msg::Int32();
        preset.data = 90;
        context->getPresetPub()->publish(preset);
        RCLCPP_INFO(context->get_logger(), "Vision task started, waiting for /vision/task_done");
    }
    else if (msg->button_id == 1)
    {
        task_active_ = false;
        context->changeState(4);
    }
}

void VisionTaskState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (task_active_)
    {
        return;
    }

    if (msg->joystick_id == 0)
    {
        auto arm_twist = geometry_msgs::msg::Twist();
        arm_twist.linear.x = applyDeadzone(msg->x, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        arm_twist.linear.y = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        context->getArmCmdPub()->publish(arm_twist);
    }
    else if (msg->joystick_id == 1)
    {
        auto arm_twist = geometry_msgs::msg::Twist();
        arm_twist.linear.z = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        arm_twist.angular.z = applyDeadzone(msg->x, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        context->getArmCmdPub()->publish(arm_twist);
    }
    else if (msg->joystick_id == 2)
    {
        auto arm_twist = geometry_msgs::msg::Twist();
        arm_twist.angular.y = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        context->getArmCmdPub()->publish(arm_twist);
    }
}

void VisionTaskState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    if (task_active_)
    {
        return;
    }

    auto cmd = std_msgs::msg::String();
    if (msg->trigger_id == 0)
    {
        cmd.data = "OPEN:" + std::to_string(-msg->value);
        context->getGripperCmdPub()->publish(cmd);
    }
    else if (msg->trigger_id == 1)
    {
        cmd.data = "CLOSE:" + std::to_string(-msg->value);
        context->getGripperCmdPub()->publish(cmd);
    }
}

void VisionTaskState::update(RobotStateMachineNode *context)
{
    if (task_active_)
    {
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        if (context->consumeVisionTaskDone())
        {
            task_active_ = false;
            RCLCPP_INFO(context->get_logger(), "Vision task done, returning to MENU");
            context->changeState(4);
        }
    }
}

double VisionTaskState::applyDeadzone(double value, double deadzone) const
{
    return std::abs(value) < deadzone ? 0.0 : value;
}
