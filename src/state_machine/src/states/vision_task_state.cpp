#include "state_machine/vision_task_state.hpp"

#include <cmath>

#include "custom_interfaces/msg/arm_pose_target.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "state_machine/robot_state_machine_node.hpp"

namespace
{
constexpr const char *kVisionCameraAppId = "mygo_pipeline_uart";
constexpr const char *kExitConfirmWord = "EXIT_NOW";
}

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
    context->setMenuItems({"A开始视觉任务", "B结束视觉程序并返回菜单"});
    context->setMenuSelection(0);

    auto camera_start = std_msgs::msg::String();
    camera_start.data = std::string("id:") + kVisionCameraAppId;
    context->getCameraStartAppPub()->publish(camera_start);

    RCLCPP_INFO(context->get_logger(), "Entered VISION_TASK state");
    RCLCPP_INFO(context->get_logger(), "Requested camera app start: %s", kVisionCameraAppId);
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

        auto camera_exit = std_msgs::msg::String();
        camera_exit.data = std::string("confirm:") + kExitConfirmWord + ",id:" + kVisionCameraAppId;
        context->getCameraExitAppPub()->publish(camera_exit);
        RCLCPP_INFO(context->get_logger(), "Requested camera app exit: %s", kVisionCameraAppId);

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
        auto pose_cmd = custom_interfaces::msg::ArmPoseTarget();
        pose_cmd.x = applyDeadzone(msg->x, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        pose_cmd.y = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        pose_cmd.z = 0.0;
        pose_cmd.roll = 0.0;
        pose_cmd.pitch = 0.0;
        pose_cmd.yaw = 0.0;
        pose_cmd.cartesian_path = false;
        context->getArmPoseTargetPub()->publish(pose_cmd);
    }
    else if (msg->joystick_id == 1)
    {
        auto pose_cmd = custom_interfaces::msg::ArmPoseTarget();
        pose_cmd.x = 0.0;
        pose_cmd.y = 0.0;
        pose_cmd.z = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        pose_cmd.roll = 0.0;
        pose_cmd.pitch = 0.0;
        pose_cmd.yaw = applyDeadzone(msg->x, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        pose_cmd.cartesian_path = false;
        context->getArmPoseTargetPub()->publish(pose_cmd);
    }
    else if (msg->joystick_id == 2)
    {
        auto pose_cmd = custom_interfaces::msg::ArmPoseTarget();
        pose_cmd.x = 0.0;
        pose_cmd.y = 0.0;
        pose_cmd.z = 0.0;
        pose_cmd.roll = 0.0;
        pose_cmd.pitch = applyDeadzone(msg->y, context->getJoystickDeadzone()) * context->getArmSpeedScale();
        pose_cmd.yaw = 0.0;
        pose_cmd.cartesian_path = false;
        context->getArmPoseTargetPub()->publish(pose_cmd);
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

    if (msg->trigger_id == 0)
    {
        auto cmd = custom_interfaces::msg::GripperCommand();
        cmd.open = true;
        context->getArmGripperCmdPub()->publish(cmd);
    }
    else if (msg->trigger_id == 1)
    {
        auto cmd = custom_interfaces::msg::GripperCommand();
        cmd.open = false;
        context->getArmGripperCmdPub()->publish(cmd);
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
