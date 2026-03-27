#include "state_machine/vision_task_state.hpp"

#include <array>

#include "custom_interfaces/msg/arm_joint_target.hpp"
#include "state_machine/robot_state_machine_node.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::array<double, 5> kVisionInitPwms = {
    1500.0, 1350.0, 2300.0, 1500.0, 1500.0};

double pwmToRad(double pwm)
{
    const double degree = (pwm - 1500.0) / 1000.0 * 135.0;
    return degree * kPi / 180.0;
}
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
    context->setMenuItems({"A开始识别", "B结束识别并返回菜单"});
    context->setMenuSelection(0);

    auto arm_target = custom_interfaces::msg::ArmJointTarget();
    arm_target.joints.reserve(kVisionInitPwms.size());
    for (const double pwm : kVisionInitPwms)
    {
        arm_target.joints.push_back(pwmToRad(pwm));
    }
    context->getArmJointTargetPub()->publish(arm_target);
    RCLCPP_INFO(
        context->get_logger(),
        "Sent vision arm init joint target: [%.3f, %.3f, %.3f, %.3f, %.3f]",
        arm_target.joints[0],
        arm_target.joints[1],
        arm_target.joints[2],
        arm_target.joints[3],
        arm_target.joints[4]);

    auto camera_stop = std_msgs::msg::String();
    camera_stop.data = "stop";
    context->getCameraVisionStopPub()->publish(camera_stop);

    RCLCPP_INFO(context->get_logger(), "Entered VISION_TASK state");
    RCLCPP_INFO(context->get_logger(), "Requested camera vision task reset to STOPPED");
}

void VisionTaskState::onExit(RobotStateMachineNode *context)
{
    context->sendStopCommands();
    auto camera_stop = std_msgs::msg::String();
    camera_stop.data = "stop";
    context->getCameraVisionStopPub()->publish(camera_stop);
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
        auto camera_start = std_msgs::msg::String();
        camera_start.data = "start";
        context->getCameraVisionStartPub()->publish(camera_start);
        RCLCPP_INFO(context->get_logger(), "Requested camera vision task START");
        return;
    }

    if (msg->button_id == 1)
    {
        auto camera_stop = std_msgs::msg::String();
        camera_stop.data = "stop";
        context->getCameraVisionStopPub()->publish(camera_stop);
        RCLCPP_INFO(context->get_logger(), "Requested camera vision task STOP");

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
