#include "state_machine/pole_state.hpp"

#include <cmath>

#include "custom_interfaces/msg/pole_command.hpp"
#include "state_machine/robot_state_machine_node.hpp"
#include "std_msgs/msg/u_int8.hpp"

std::string PoleState::getName() const
{
    return "POLE";
}

uint8_t PoleState::getStateEnum() const
{
    return 5;
}

uint8_t PoleState::getSubState() const
{
    const bool selected_locked = pole_locked_[static_cast<size_t>(selected_id_)] ;
    return static_cast<uint8_t>((selected_locked ? 2U : 0U) | (selected_id_ & 0x01U));
}

std::vector<std::string> PoleState::getAvailableModes() const
{
    return {
        pole_locked_[0] ? "LEFT_LOCKED" : "LEFT_UNLOCKED",
        pole_locked_[1] ? "RIGHT_LOCKED" : "RIGHT_UNLOCKED"
    };
}

void PoleState::onEnter(RobotStateMachineNode *context)
{
    pole_locked_ = {false, false};
    rt_pressed_ = false;
    selected_id_ = 0;
    context->setMenuItems({"退出杆子模式", "LB->ID0", "RB->ID1", "RT锁定/解锁"});
    context->setMenuSelection(0);
    publishSelectedId(context);
    publishLockMask(context);
    RCLCPP_INFO(context->get_logger(), "Entered POLE state");
}

void PoleState::onExit(RobotStateMachineNode *context)
{
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
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
        return;
    }

    if (msg->button_id == 4)
    {
        selected_id_ = 0;
        publishSelectedId(context);
        RCLCPP_INFO(context->get_logger(), "POLE selected servo id: %u", selected_id_);
        return;
    }

    if (msg->button_id == 5)
    {
        selected_id_ = 1;
        publishSelectedId(context);
        RCLCPP_INFO(context->get_logger(), "POLE selected servo id: %u", selected_id_);
    }
}

void PoleState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 0)
    {
        const auto x = applyDeadzone(msg->x, context->getJoystickDeadzone());
        const auto y = applyDeadzone(msg->y, context->getJoystickDeadzone());

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = y * context->getChassisMaxLinearSpeed();
        twist.angular.z = -x * context->getChassisMaxAngularSpeed();
        context->getChassisCmdPub()->publish(twist);
        return;
    }

    if (msg->joystick_id != 1 || pole_locked_[static_cast<size_t>(selected_id_)])
    {
        return;
    }

    const float delta = static_cast<float>(applyDeadzone(msg->y, context->getJoystickDeadzone()));
    if (std::abs(delta) < 1e-4F)
    {
        return;
    }

    auto cmd = custom_interfaces::msg::PoleCommand();
    cmd.id = (selected_id_ != 0);
    cmd.delta = delta;
    context->getPoleRotatePub()->publish(cmd);
}

void PoleState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    if (msg->trigger_id != 1)
    {
        return;
    }

    const bool currently_pressed = (msg->event_type == 1) || (msg->value < -0.8F);
    if (currently_pressed && !rt_pressed_)
    {
        auto &selected_locked = pole_locked_[static_cast<size_t>(selected_id_)];
        selected_locked = !selected_locked;
        publishLockMask(context);
        RCLCPP_INFO(
            context->get_logger(),
            "POLE servo %u %s",
            selected_id_,
            selected_locked ? "LOCKED" : "UNLOCKED");
    }

    rt_pressed_ = currently_pressed;
}

void PoleState::update(RobotStateMachineNode *context)
{
    (void)context;
}

double PoleState::applyDeadzone(double value, double deadzone) const
{
    return std::abs(value) < deadzone ? 0.0 : value;
}

void PoleState::publishSelectedId(RobotStateMachineNode *context) const
{
    auto msg = std_msgs::msg::UInt8();
    msg.data = selected_id_;
    context->getPoleSelectedIdPub()->publish(msg);
}

void PoleState::publishLockMask(RobotStateMachineNode *context) const
{
    auto msg = std_msgs::msg::UInt8();
    uint8_t mask = 0;
    if (pole_locked_[0]) {
        mask |= 0x01U;
    }
    if (pole_locked_[1]) {
        mask |= 0x02U;
    }
    msg.data = mask;
    context->getPoleLockMaskPub()->publish(msg);
}
