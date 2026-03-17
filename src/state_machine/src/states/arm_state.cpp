#include "state_machine/arm_state.hpp"

#include <algorithm>
#include <cmath>

#include "custom_interfaces/msg/arm_named_target.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "state_machine/robot_state_machine_node.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr int kButtonLB = 4;
constexpr int kButtonRB = 5;
constexpr uint8_t kPressEvent = 0;
constexpr uint8_t kReleaseEvent = 1;
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
    precision_mode_ = false;
    right_y_selected_servo_ = 2;
    dpad_switch_latched_ = false;
    rt_press_latched_ = false;
    updateSubmenuUi(context);

    if (!joint_state_sub_)
    {
        joint_state_sub_ = context->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 20,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                if (msg->position.size() < target_joints_rad_.size())
                {
                    return;
                }
                for (size_t i = 0; i < target_joints_rad_.size(); ++i)
                {
                    target_joints_rad_[i] = msg->position[i];
                }
                joint_state_received_ = true;
                target_joints_initialized_ = true;
            });
    }

    syncJointStateOnce(context);
    last_update_time_ = context->now();

    RCLCPP_INFO(
        context->get_logger(),
        "Entered ARM state: default HIGH_SPEED mode, right-stick-y controls servo %d",
        right_y_selected_servo_);
}

void ArmState::onExit(RobotStateMachineNode *context)
{
    (void)context;
}

void ArmState::handleButton(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
{
    if (msg->event_type != kPressEvent)
    {
        if (msg->button_id == kButtonLB && msg->event_type == kReleaseEvent && submenu_active_)
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

    if (msg->button_id == kButtonLB)
    {
        submenu_active_ = true;
        updateSubmenuUi(context);
        return;
    }

    if (msg->button_id == kButtonRB)
    {
        precision_mode_ = !precision_mode_;
        RCLCPP_INFO(
            context->get_logger(),
            "ARM control mode switched to: %s",
            precision_mode_ ? "PRECISION" : "HIGH_SPEED");
        return;
    }

    if (msg->button_id == 0)
    {
        auto gripper_cmd = custom_interfaces::msg::GripperCommand();
        gripper_cmd.open = false;
        gripper_open_ = false;
        context->getArmGripperCmdPub()->publish(gripper_cmd);
        return;
    }

    if (msg->button_id == 1)
    {
        auto gripper_cmd = custom_interfaces::msg::GripperCommand();
        gripper_cmd.open = true;
        gripper_open_ = true;
        context->getArmGripperCmdPub()->publish(gripper_cmd);
    }
}

void ArmState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (msg->joystick_id == 1 && submenu_active_)
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

    if (submenu_active_)
    {
        return;
    }

    if (msg->joystick_id == 2)
    {
        const float y = msg->y;
        const float threshold = 0.6F;
        const float reset_threshold = 0.2F;

        if (!dpad_switch_latched_ && y >= threshold)
        {
            right_y_selected_servo_ = 2;
            dpad_switch_latched_ = true;
            RCLCPP_INFO(context->get_logger(), "ARM right-stick-y target switched to servo 2");
        }
        else if (!dpad_switch_latched_ && y <= -threshold)
        {
            right_y_selected_servo_ = 3;
            dpad_switch_latched_ = true;
            RCLCPP_INFO(context->get_logger(), "ARM right-stick-y target switched to servo 3");
        }
        else if (std::abs(y) < reset_threshold)
        {
            dpad_switch_latched_ = false;
        }
    }
}

void ArmState::handleTrigger(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::TriggerIntent::SharedPtr msg)
{
    if (msg->trigger_id != 1)
    {
        return;
    }

    if (msg->event_type == 1 && !rt_press_latched_)
    {
        rt_press_latched_ = true;
        gripper_open_ = !gripper_open_;

        auto gripper_cmd = custom_interfaces::msg::GripperCommand();
        gripper_cmd.open = gripper_open_;
        context->getArmGripperCmdPub()->publish(gripper_cmd);

        RCLCPP_INFO(
            context->get_logger(),
            "RT pressed -> gripper toggled: %s",
            gripper_open_ ? "OPEN" : "CLOSE");
    }
    else if (msg->event_type == 0)
    {
        rt_press_latched_ = false;
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
    if (submenu_active_)
    {
        return;
    }

    syncJointStateOnce(context);
    if (!target_joints_initialized_)
    {
        return;
    }

    auto now = context->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    if (dt <= 0.0 || dt > 0.2)
    {
        const int hz = 50;
        dt = 1.0 / static_cast<double>(hz);
    }

    applyAxisControl(context, dt);
    publishJointCommand(context);
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

void ArmState::publishJointCommand(RobotStateMachineNode *context)
{
    auto msg = example_interfaces::msg::Float64MultiArray();
    msg.data.assign(target_joints_rad_.begin(), target_joints_rad_.end());
    context->getArmJointCommandPub()->publish(msg);
}

void ArmState::syncJointStateOnce(RobotStateMachineNode *context)
{
    if (target_joints_initialized_)
    {
        return;
    }

    if (joint_state_received_)
    {
        target_joints_initialized_ = true;
        return;
    }

    const double center_rad = 135.0 * kPi / 180.0;
    for (auto &joint : target_joints_rad_)
    {
        joint = center_rad;
    }
    target_joints_initialized_ = true;

    RCLCPP_WARN_THROTTLE(
        context->get_logger(),
        *context->get_clock(),
        2000,
        "No /joint_states received yet, using 135deg fallback for arm control.");
}

void ArmState::applyAxisControl(RobotStateMachineNode *context, double dt)
{
    const auto &left = context->getLeftJoystick();
    const auto &right = context->getRightJoystick();
    const double deadzone = context->getJoystickDeadzone();

    const double lx = (std::abs(left.x) < deadzone) ? 0.0 : static_cast<double>(left.x);
    const double ly = (std::abs(left.y) < deadzone) ? 0.0 : static_cast<double>(left.y);
    const double rx = (std::abs(right.x) < deadzone) ? 0.0 : static_cast<double>(right.x);
    const double ry = (std::abs(right.y) < deadzone) ? 0.0 : static_cast<double>(right.y);

    const auto &speed = precision_mode_ ? max_speed_precision_rad_s_ : max_speed_high_rad_s_;

    target_joints_rad_[0] += speed[0] * lx * dt;
    target_joints_rad_[1] += speed[1] * ly * dt;
    target_joints_rad_[4] += speed[4] * rx * dt;
    target_joints_rad_[right_y_selected_servo_] += speed[right_y_selected_servo_] * ry * dt;

    for (size_t i = 0; i < target_joints_rad_.size(); ++i)
    {
        target_joints_rad_[i] = std::clamp(target_joints_rad_[i], min_joint_rad_[i], max_joint_rad_[i]);
    }
}

