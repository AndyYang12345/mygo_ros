#include "state_machine/arm_state.hpp"

#include <cmath>

#include "custom_interfaces/msg/arm_named_target.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "state_machine/robot_state_machine_node.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846F;
const char *kOctantNames[8] = {
    "RIGHT", "UP_RIGHT", "UP", "UP_LEFT", "LEFT", "DOWN_LEFT", "DOWN", "DOWN_RIGHT"};
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
    submenu_active_ = false;
    submenu_selection_ = 0;
    updateSubmenuUi(context);
    RCLCPP_INFO(context->get_logger(), "Entered ARM state");
}

void ArmState::onExit(RobotStateMachineNode *context)
{
    (void)context;
}

void ArmState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != 0)
    {
        if (msg->button_id == 4 && msg->event_type == 1 && submenu_active_)
        {
            submenu_active_ = false;
            auto target = custom_interfaces::msg::ArmNamedTarget();
            if (submenu_selection_ >= 0 && submenu_selection_ < static_cast<int>(presets_.size()))
            {
                target.target_name = presets_[submenu_selection_];
            }
            else
            {
                target.target_name = "home";
            }
            context->getArmNamedTargetPub()->publish(target);
            updateSubmenuUi(context);
            RCLCPP_INFO(
                context->get_logger(),
                "ARM submenu selected index %d -> named target: %s",
                submenu_selection_,
                target.target_name.c_str());
        }
        return;
    }

    if (msg->button_id == 4)
    {
        submenu_active_ = true;
        updateSubmenuUi(context);
        return;
    }

    if (msg->button_id == 0)
    {
        auto gripper_cmd = custom_interfaces::msg::GripperCommand();
        gripper_cmd.open = false;
        context->getArmGripperCmdPub()->publish(gripper_cmd);
        return;
    }

    if (msg->button_id == 1)
    {
        auto gripper_cmd = custom_interfaces::msg::GripperCommand();
        gripper_cmd.open = true;
        context->getArmGripperCmdPub()->publish(gripper_cmd);
    }
}

void ArmState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 1)
    {
        if (submenu_active_)
        {
            const float x = msg->x;
            const float y = msg->y;
            const float submenu_deadzone = 0.25F;
            const float radius = std::sqrt(x * x + y * y);
            if (radius >= submenu_deadzone)
            {
                const int octant = angleToOctant(x, y);
                if (octant != submenu_selection_)
                {
                    submenu_selection_ = octant;
                    updateSubmenuUi(context);

                    const float norm_x = -x;
                    const float norm_y = y;
                    const float angle_rad = std::atan2(norm_y, norm_x);
                    const float angle_deg = angle_rad * 180.0F / kPi;

                    std::string mapped_target = "home";
                    if (submenu_selection_ >= 0 && submenu_selection_ < static_cast<int>(presets_.size()))
                    {
                        mapped_target = presets_[submenu_selection_];
                    }

                    RCLCPP_INFO(
                        context->get_logger(),
                        "[ARM_SUBMENU] angle=%.1f deg, octant=%s -> index=%d, item=%s, named_target=%s",
                        angle_deg,
                        kOctantNames[submenu_selection_],
                        submenu_selection_,
                        presets_[submenu_selection_].c_str(),
                        mapped_target.c_str());
                }
            }
            return;
        }
    }

    (void)context;
}

void ArmState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    (void)context;
    (void)msg;
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
    if (submenu_active_)
    {
        return;
    }

    (void)context;
}

uint8_t ArmState::getSubState() const
{
    return static_cast<uint8_t>(submenu_selection_);
}

std::vector<std::string> ArmState::getAvailableModes() const
{
    return presets_;
}

void ArmState::updateSubmenuUi(RobotStateMachineNode *context)
{
    context->setMenuItems(presets_);
    context->setMenuSelection(submenu_selection_);
}

int ArmState::angleToOctant(float x, float y) const
{
    const float norm_x = -x;
    const float norm_y = y;
    float angle = std::atan2(norm_y, norm_x);
    if (angle < 0.0F)
    {
        angle += static_cast<float>(2.0 * kPi);
    }

    const float sector = static_cast<float>((2.0 * kPi) / 8.0);
    int octant = static_cast<int>(std::floor((angle + sector * 0.5F) / sector));
    octant %= 8;
    return octant;
}

