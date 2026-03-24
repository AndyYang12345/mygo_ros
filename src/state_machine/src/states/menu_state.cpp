#include "state_machine/menu_state.hpp"

#include "state_machine/robot_state_machine_node.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846F;
const char *kOctantNames[8] = {
    "RIGHT", "UP_RIGHT", "UP", "UP_LEFT", "LEFT", "DOWN_LEFT", "DOWN", "DOWN_RIGHT"};
}

std::vector<std::string> MenuState::getAvailableModes() const
{
    std::vector<std::string> names;
    names.reserve(menu_entries_.size());
    for (const auto &entry : menu_entries_)
    {
        names.push_back(entry.first);
    }
    return names;
}

void MenuState::onEnter(RobotStateMachineNode *context)
{
    trigger_pressed_ = false;
    selection_touched_ = false;
    selection_index_ = 0;
    last_activity_ = context->now();
    refreshMenuState(context);
    RCLCPP_INFO(context->get_logger(), "Entered MENU mode");
}

void MenuState::onExit(RobotStateMachineNode *context)
{
    (void)context;
}

void MenuState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    if (msg->trigger_id != 0)
    {
        return;
    }

    const bool currently_pressed = (msg->value < -0.15F) || (msg->event_type == 1);

    if (currently_pressed && !trigger_pressed_)
    {
        trigger_pressed_ = true;
        last_activity_ = context->now();
        return;
    }

    if (!currently_pressed && trigger_pressed_)
    {
        trigger_pressed_ = false;
        if (!selection_touched_)
        {
            context->changeState(context->getStateBeforeMenu());
            return;
        }
        const uint8_t target_state = menu_entries_[selection_index_].second;
        context->changeState(target_state);
    }
}

void MenuState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id != 1 || !trigger_pressed_)
    {
        return;
    }

    const float x = msg->x;
    const float y = msg->y;
    const float deadzone = 0.25F;
    const float radius = std::sqrt(x * x + y * y);

    if (radius < deadzone)
    {
        RCLCPP_INFO_THROTTLE(
            context->get_logger(), *context->get_clock(), 200,
            "[MENU] joystick centered (x=%.2f, y=%.2f), keep index=%d, item=%s",
            x, y, selection_index_, menu_entries_[selection_index_].first.c_str());
        return;
    }

    const int octant = angleToOctant(x, y);
    const int new_index = octantToMenuIndex(octant);
    const float norm_x = -x;
    const float norm_y = y;
    const float angle_rad = std::atan2(norm_y, norm_x);
    const float angle_deg = angle_rad * 180.0F / kPi;
    selection_touched_ = true;
    last_activity_ = context->now();

    if (new_index != selection_index_)
    {
        selection_index_ = new_index;
        refreshMenuState(context);
        RCLCPP_INFO(
            context->get_logger(),
            "[MENU] angle=%.1f deg, octant=%s -> index=%d, item=%s",
            angle_deg,
            kOctantNames[octant],
            selection_index_,
            menu_entries_[selection_index_].first.c_str());
    }
    else
    {
        RCLCPP_INFO_THROTTLE(
            context->get_logger(), *context->get_clock(), 150,
            "[MENU] angle=%.1f deg, octant=%s, index=%d, item=%s",
            angle_deg,
            kOctantNames[octant],
            selection_index_,
            menu_entries_[selection_index_].first.c_str());
    }
}

void MenuState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0)
    {
        return;
    }

    if (msg->button_id == 1)
    {
        context->changeState(context->getStateBeforeMenu());
    }
}

void MenuState::update(RobotStateMachineNode *context)
{
    if (!trigger_pressed_)
    {
        return;
    }

    if ((context->now() - last_activity_).seconds() > 8.0)
    {
        context->changeState(context->getStateBeforeMenu());
    }
}

void MenuState::refreshMenuState(RobotStateMachineNode *context)
{
    auto names = getAvailableModes();
    context->setMenuItems(names);
    context->setMenuSelection(selection_index_);
}

int MenuState::angleToOctant(float x, float y) const
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

int MenuState::octantToMenuIndex(int octant) const
{
    static constexpr int kOctantToMenu[8] = {
        1,  // RIGHT -> ARM
        3,  // UP_RIGHT -> VISION_TASK
        0,  // UP -> CHASSIS
        5,  // UP_LEFT -> POLE
        4,  // LEFT -> IDLE
        2,  // DOWN_LEFT -> BALL
        0,  // DOWN -> CHASSIS
        1   // DOWN_RIGHT -> ARM
    };

    if (octant < 0 || octant >= 8)
    {
        return selection_index_;
    }

    const int mapped = kOctantToMenu[octant];
    if (mapped < 0 || mapped >= static_cast<int>(menu_entries_.size()))
    {
        return selection_index_;
    }
    return mapped;
}
