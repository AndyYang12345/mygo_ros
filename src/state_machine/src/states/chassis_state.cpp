#include "state_machine/chassis_state.hpp"
#include "state_machine/robot_state_machine_node.hpp"

#include <algorithm>
#include <cmath>

#include "custom_interfaces/msg/arm_named_target.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr bool kChassisDebugEnabled = true;
constexpr float kPi = 3.14159265358979323846F;
}

ChassisState::ChassisState()
{
    submenu_items_ = {
        {"MD", &ChassisState::publishCollectorMiddleCommand},
        {"-", &ChassisState::publishNoOpCommand},
        {"DN", &ChassisState::publishCollectorDownCommand},
        {"-", &ChassisState::publishNoOpCommand},
        {"under_bridge", &ChassisState::publishArmUnderBridgeCommand},
        {"-", &ChassisState::publishNoOpCommand},
        {"folded", &ChassisState::publishArmFoldedCommand},
        {"-", &ChassisState::publishNoOpCommand},
    };
}

std::string ChassisState::getName() const
{
    return "CHASSIS";
}

uint8_t ChassisState::getStateEnum() const
{
    return 2;
}

uint8_t ChassisState::getSubState() const
{
    return static_cast<uint8_t>(submenu_selection_);
}

std::vector<std::string> ChassisState::getAvailableModes() const
{
    std::vector<std::string> modes;
    modes.reserve(submenu_items_.size());
    for (const auto &item : submenu_items_)
    {
        modes.push_back(item.label);
    }
    return modes;
}

void ChassisState::onEnter(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "Entered CHASSIS state");
    submenu_active_ = false;
    submenu_selection_ = 0;
    updateSubmenuUi(context);
}

void ChassisState::onExit(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "Exiting CHASSIS state");
    context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
    context->getArmCmdPub()->publish(geometry_msgs::msg::Twist());
}

void ChassisState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (submenu_active_ && msg->joystick_id == 1)
    {
        const float x = msg->x;
        const float y = msg->y;
        const float deadzone = 0.25F;
        const float radius = std::sqrt(x * x + y * y);

        if (radius < deadzone)
        {
            RCLCPP_INFO_THROTTLE(
                context->get_logger(), *context->get_clock(), 200,
                "[CHASSIS_SUBMENU] joystick centered (x=%.2f, y=%.2f), keep index=%d, item=%s",
                x,
                y,
                submenu_selection_,
                submenu_items_[static_cast<size_t>(submenu_selection_)].label);
            return;
        }

        const int octant = angleToOctant(x, y);
        if (octant != submenu_selection_)
        {
            submenu_selection_ = octant;
            updateSubmenuUi(context);
            RCLCPP_INFO(
                context->get_logger(),
                "[CHASSIS_SUBMENU] octant=%d -> index=%d, item=%s",
                octant,
                submenu_selection_,
                submenu_items_[static_cast<size_t>(submenu_selection_)].label);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                context->get_logger(), *context->get_clock(), 150,
                "[CHASSIS_SUBMENU] octant=%d, index=%d, item=%s",
                octant,
                submenu_selection_,
                submenu_items_[static_cast<size_t>(submenu_selection_)].label);
        }
        return;
    }

    if (msg->joystick_id == 0)
    {
        processChassisControl(context);
    }
}

void ChassisState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    // 仅保留子菜单交互：LB按下打开，LB松开执行。
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
            const auto index = static_cast<size_t>(std::clamp(
                submenu_selection_, 0, static_cast<int>(submenu_items_.size() - 1)));
            auto handler = submenu_items_[index].handler;
            (this->*handler)(context);
            updateSubmenuUi(context);
            return;
        }
    }

    // 子菜单打开期间允许B取消。
    if (submenu_active_ && msg->button_id == 1 && msg->event_type == 0)
    {
        submenu_active_ = false;
        updateSubmenuUi(context);
    }
}

void ChassisState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    (void)context;
    (void)msg;
    // CHASSIS状态中不处理扳机映射。
}

void ChassisState::update(RobotStateMachineNode *context)
{
    processChassisControl(context);
}

double ChassisState::applyDeadzone(double value, double deadzone) const
{
    return std::abs(value) < deadzone ? 0.0 : value;
}

void ChassisState::publishCollectorCommand(RobotStateMachineNode *context, const std::string &command)
{
    auto msg = std_msgs::msg::String();
    msg.data = command;
    context->getCollectorCmdPub()->publish(msg);
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> collector command: %s", command.c_str());
}

void ChassisState::publishArmUnderBridgeCommand(RobotStateMachineNode *context)
{
    auto target = custom_interfaces::msg::ArmNamedTarget();
    target.target_name = "under_bridge";
    context->getArmNamedTargetPub()->publish(target);
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> arm named target: under_bridge");
}

void ChassisState::publishArmFoldedCommand(RobotStateMachineNode *context)
{
    auto target = custom_interfaces::msg::ArmNamedTarget();
    target.target_name = "folded";
    context->getArmNamedTargetPub()->publish(target);
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> arm named target: folded");
}

void ChassisState::publishCollectorMiddleCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "MD");
}

void ChassisState::publishCollectorDownCommand(RobotStateMachineNode *context)
{
    publishCollectorCommand(context, "DN");
}

void ChassisState::publishNoOpCommand(RobotStateMachineNode *context)
{
    RCLCPP_INFO(context->get_logger(), "CHASSIS submenu -> no-op item selected");
}

void ChassisState::processChassisControl(RobotStateMachineNode *context)
{
    const auto &joystick = context->getLeftJoystick();

    if (!joystick.is_active(context))
    {
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        return;
    }

    const double x = applyDeadzone(joystick.x, context->getJoystickDeadzone());
    const double y = applyDeadzone(joystick.y, context->getJoystickDeadzone());

    if (x == 0.0 && y == 0.0)
    {
        context->getChassisCmdPub()->publish(geometry_msgs::msg::Twist());
        return;
    }

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = y * context->getChassisMaxLinearSpeed();
    twist.angular.z = x * context->getChassisMaxAngularSpeed();
    context->getChassisCmdPub()->publish(twist);

    if (kChassisDebugEnabled)
    {
        RCLCPP_DEBUG(
            context->get_logger(),
            "Chassis cmd: linear=%.2f m/s, angular=%.2f rad/s",
            twist.linear.x,
            twist.angular.z);
    }
}

void ChassisState::updateSubmenuUi(RobotStateMachineNode *context)
{
    std::vector<std::string> names;
    names.reserve(submenu_items_.size());
    for (const auto &item : submenu_items_)
    {
        names.push_back(item.label);
    }

    context->setMenuItems(names);

    if (submenu_active_)
    {
        context->setMenuSelection(submenu_selection_);
        return;
    }

    context->setMenuSelection(std::clamp(
        submenu_selection_, 0, static_cast<int>(submenu_items_.size() - 1)));
}

int ChassisState::angleToOctant(float x, float y) const
{
    const float norm_x = -x;
    const float norm_y = y;

    float angle = std::atan2(norm_y, norm_x);
    if (angle < 0.0F)
    {
        angle += 2.0F * kPi;
    }

    const float sector = (2.0F * kPi) / 8.0F;
    int octant = static_cast<int>(std::floor((angle + sector * 0.5F) / sector));
    octant %= 8;
    return octant;
}
