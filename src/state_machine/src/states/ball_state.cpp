#include "state_machine/ball_state.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "state_machine/robot_state_machine_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr float kSubmenuDeadzone = 0.25F;
constexpr int kOctantCount = 8;

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
    submenu_active_ = false;
    submenu_selection_ = 0;
    submenu_items_ = {
        {"NEXT", &BallState::publishNextCollectorCommand},
        {"MD", &BallState::publishCollectorMiddleCommand},
        {"DN", &BallState::publishCollectorDownCommand},
        {"UP", &BallState::publishCollectorUpCommand},
        {"OP", &BallState::publishCollectorOpenCommand},
        {"CL", &BallState::publishCollectorCloseCommand},
        {"-", &BallState::publishNoOpCommand},
        {"-", &BallState::publishNoOpCommand},
    };
    updateSubmenuUi(context);
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

    if (msg->button_id == 4)
    {
        if (msg->event_type == 0)
        {
            submenu_active_ = true;
            submenu_selection_ = 0;
            updateSubmenuUi(context);
            return;
        }

        if (msg->event_type == 1 && submenu_active_)
        {
            submenu_active_ = false;
            const auto index = static_cast<size_t>(std::clamp(submenu_selection_, 0, static_cast<int>(submenu_items_.size() - 1)));
            auto handler = submenu_items_[index].handler;
            (this->*handler)(context);
            updateSubmenuUi(context);
            return;
        }
    }

    if (submenu_active_)
    {
        if (msg->button_id == 1 && msg->event_type == 0)
        {
            submenu_active_ = false;
            updateSubmenuUi(context);
        }
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

void BallState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (!submenu_active_ || msg->joystick_id != 1)
    {
        return;
    }

    const float x = msg->x;
    const float y = msg->y;
    const float radius = std::sqrt(x * x + y * y);
    if (radius < kSubmenuDeadzone)
    {
        RCLCPP_INFO_THROTTLE(
            context->get_logger(), *context->get_clock(), 200,
            "[BALL_SUBMENU] joystick centered (x=%.2f, y=%.2f), keep index=%d, item=%s",
            x, y, submenu_selection_, submenu_items_[static_cast<size_t>(submenu_selection_)].label);
        return;
    }

    const int octant = angleToOctant(x, y);
    if (octant != submenu_selection_)
    {
        submenu_selection_ = octant;
        updateSubmenuUi(context);
        RCLCPP_INFO(
            context->get_logger(),
            "[BALL_SUBMENU] octant=%d -> index=%d, item=%s",
            octant,
            submenu_selection_,
            submenu_items_[static_cast<size_t>(submenu_selection_)].label);
    }
    else
    {
        RCLCPP_INFO_THROTTLE(
            context->get_logger(), *context->get_clock(), 150,
            "[BALL_SUBMENU] octant=%d, index=%d, item=%s",
            octant,
            submenu_selection_,
            submenu_items_[static_cast<size_t>(submenu_selection_)].label);
    }
}

uint8_t BallState::getSubState() const
{
    return static_cast<uint8_t>(submenu_selection_);
}

std::vector<std::string> BallState::getAvailableModes() const
{
    std::vector<std::string> names;
    names.reserve(submenu_items_.size());
    for (const auto &item : submenu_items_)
    {
        names.push_back(item.label);
    }
    return names;
}

void BallState::update(RobotStateMachineNode *context)
{
    if (submenu_active_)
    {
        return;
    }

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

void BallState::updateSubmenuUi(RobotStateMachineNode *context)
{
    std::vector<std::string> names;
    names.reserve(submenu_items_.size());
    for (const auto &item : submenu_items_)
    {
        names.push_back(item.label);
    }

    context->setMenuItems(names);
    context->setMenuSelection(submenu_selection_);
}

int BallState::angleToOctant(float x, float y) const
{
    const float norm_x = -x;
    const float norm_y = y;

    float angle = std::atan2(norm_y, norm_x);
    if (angle < 0.0F)
    {
        angle += 2.0F * kPi;
    }

    const float sector = (2.0F * kPi) / static_cast<float>(kOctantCount);
    int octant = static_cast<int>(std::floor((angle + sector * 0.5F) / sector));
    octant %= kOctantCount;
    return octant;
}

void BallState::publishCollectorCommand(RobotStateMachineNode *context, const std::string &command)
{
    auto trigger = std_msgs::msg::String();
    trigger.data = command;
    context->getCollectorCmdPub()->publish(trigger);
    RCLCPP_INFO(context->get_logger(), "BALL submenu -> collector command: %s", command.c_str());
}

void BallState::publishNextCollectorCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "NEXT");
}

void BallState::publishCollectorMiddleCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "MD");
}

void BallState::publishCollectorDownCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "DN");
}

void BallState::publishCollectorUpCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "UP");
}

void BallState::publishCollectorOpenCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "OP");
}

void BallState::publishCollectorCloseCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "CL");
}

void BallState::publishNoOpCommand(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "BALL submenu -> no-op item selected");
}
