#include "state_machine/vision_task_state.hpp"

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
    return 0;
}

void VisionTaskState::onEnter(RobotStateMachineNode *context)
{
    context->setMenuItems({"B结束视觉程序并返回菜单"});
    context->setMenuSelection(0);

    auto arm_cmd = std_msgs::msg::String();
    arm_cmd.data = "{P1500T1000P1350T1000P2300T1000P1500T1000P1500T1000}";
    context->getArmQueryCurrentPub()->publish(arm_cmd);
    RCLCPP_INFO(context->get_logger(), "Sent arm init command: %s", arm_cmd.data.c_str());

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

    if (msg->button_id == 1)
    {
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
    (void)context;
    (void)msg;
}

void VisionTaskState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    (void)context;
    (void)msg;
}

void VisionTaskState::update(RobotStateMachineNode *context)
{
    (void)context;
}
