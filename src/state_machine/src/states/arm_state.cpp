#include "state_machine/arm_state.hpp"

#include <algorithm>
#include <cmath>

#include "custom_interfaces/msg/arm_named_target.hpp"
#include "custom_interfaces/msg/gripper_command.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"
#include "state_machine/robot_state_machine_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr float kPi = 3.14159265358979323846F;
constexpr int kButtonLB = 4;
constexpr int kButtonRB = 5;
constexpr uint8_t kPressEvent = 0;
constexpr uint8_t kReleaseEvent = 1;
const char *kOctantNames[8] = {
    "RIGHT", "UP_RIGHT", "UP", "UP_LEFT", "LEFT", "DOWN_LEFT", "DOWN", "DOWN_RIGHT"};

double pwmToRad(double pwm)
{
    const double degree = (pwm - 1500.0) / 1000.0 * 135.0;
    return degree * static_cast<double>(kPi) / 180.0;
}
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
    target_joints_initialized_ = false;
    waiting_initial_state_ = true;
    preset_sync_pending_ = false;
    preset_motion_in_progress_ = false;
    kg_sync_requested_ = false;
    latest_feedback_valid_ = false;
    updateSubmenuUi(context);

    if (!current_joint_sub_)
    {
        current_joint_sub_ = context->create_subscription<example_interfaces::msg::Float64MultiArray>(
            "/arm/current_pwm", 10,
            [this, context](const example_interfaces::msg::Float64MultiArray::SharedPtr msg)
            {
                if (!msg || msg->data.size() < 5)
                {
                    return;
                }

                for (size_t i = 0; i < target_pwms_.size(); ++i)
                {
                    latest_feedback_joints_rad_[i] = pwmToRad(msg->data[i]);
                }
                latest_feedback_valid_ = true;

                const bool should_update_direct_target =
                    (!target_joints_initialized_) || waiting_initial_state_ || preset_sync_pending_;

                if (should_update_direct_target)
                {
                    for (size_t i = 0; i < target_pwms_.size(); ++i)
                    {
                        target_pwms_[i] = msg->data[i];
                        target_joints_rad_[i] = latest_feedback_joints_rad_[i];
                    }
                    target_joints_initialized_ = true;
                    waiting_initial_state_ = false;
                }

                if (kg_sync_requested_)
                {
                    kg_sync_requested_ = false;
                }
            });
    }

    auto query_msg = std_msgs::msg::String();
    query_msg.data = "kg";
    context->getArmQueryCurrentPub()->publish(query_msg);
    last_query_time_ = context->now();
    next_sync_query_time_ = context->now() + rclcpp::Duration::from_seconds(kg_sync_interval_s_);

    last_update_time_ = context->now();

    RCLCPP_INFO(
        context->get_logger(),
        "Entered ARM state: sent kg query, waiting current joints, right-stick-y controls servo %d after init",
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
            target_joints_initialized_ = false;
            waiting_initial_state_ = true;
            preset_sync_pending_ = true;
            preset_motion_in_progress_ = true;
            preset_sync_due_time_ = context->now() + rclcpp::Duration::from_seconds(preset_sync_delay_s_);
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
    if (preset_motion_in_progress_)
    {
        return;
    }

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
            right_y_selected_servo_ = 3;
            dpad_switch_latched_ = true;
            RCLCPP_INFO(context->get_logger(), "ARM right-stick-y target switched to servo 3");
        }
        else if (!dpad_switch_latched_ && y <= -threshold)
        {
            right_y_selected_servo_ = 2;
            dpad_switch_latched_ = true;
            RCLCPP_INFO(context->get_logger(), "ARM right-stick-y target switched to servo 2");
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

    if (preset_motion_in_progress_)
    {
        if (context->now() < preset_sync_due_time_)
        {
            return;
        }
        preset_motion_in_progress_ = false;
    }

    if (!target_joints_initialized_)
    {
        const auto now = context->now();

        if (preset_sync_pending_ && now >= preset_sync_due_time_)
        {
            auto query_msg = std_msgs::msg::String();
            query_msg.data = "kg";
            context->getArmQueryCurrentPub()->publish(query_msg);
            last_query_time_ = now;
            preset_sync_pending_ = false;
            return;
        }

        if (!preset_sync_pending_ && (now - last_query_time_).seconds() > 0.35)
        {
            auto query_msg = std_msgs::msg::String();
            query_msg.data = "kg";
            context->getArmQueryCurrentPub()->publish(query_msg);
            last_query_time_ = now;
        }

        RCLCPP_WARN_THROTTLE(
            context->get_logger(),
            *context->get_clock(),
            1500,
            "ARM waiting kg current state, control publish paused.");
        return;
    }

    const auto now = context->now();
    if (kg_sync_requested_ && (now - last_query_time_).seconds() > kg_query_timeout_s_)
    {
        kg_sync_requested_ = false;
    }
    if (!kg_sync_requested_ && now >= next_sync_query_time_)
    {
        auto query_msg = std_msgs::msg::String();
        query_msg.data = "kg";
        context->getArmQueryCurrentPub()->publish(query_msg);
        last_query_time_ = now;
        kg_sync_requested_ = true;
        next_sync_query_time_ = now + rclcpp::Duration::from_seconds(kg_sync_interval_s_);
    }

    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    if (dt <= 0.0 || dt > 0.2)
    {
        const int hz = 50;
        dt = 1.0 / static_cast<double>(hz);
    }

    const bool changed = applyAxisControl(context, dt);
    if (!changed)
    {
        return;
    }

    publishDirectPwmCommand(context);
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

void ArmState::publishDirectPwmCommand(RobotStateMachineNode *context)
{
    auto msg = example_interfaces::msg::Float64MultiArray();
    msg.data.assign(target_pwms_.begin(), target_pwms_.end());
    context->getArmDirectPwmPub()->publish(msg);
}

bool ArmState::applyAxisControl(RobotStateMachineNode *context, double dt)
{
    const auto &left = context->getLeftJoystick();
    const auto &right = context->getRightJoystick();
    const double deadzone = context->getJoystickDeadzone();

    const double lx = (std::abs(left.x) < deadzone) ? 0.0 : static_cast<double>(left.x);
    const double ly = (std::abs(left.y) < deadzone) ? 0.0 : static_cast<double>(left.y);
    const double rx = (std::abs(right.x) < deadzone) ? 0.0 : static_cast<double>(right.x);
    const double ry = (std::abs(right.y) < deadzone) ? 0.0 : static_cast<double>(right.y);

    const auto &speed = precision_mode_ ? max_speed_precision_pwm_s_ : max_speed_high_pwm_s_;

    std::array<double, 5> old_pwms = target_pwms_;
    const std::array<double, 5> delta = {
        speed[0] * lx * dt,
        speed[1] * ly * dt,
        speed[2] * 0.0 * dt,
        speed[3] * 0.0 * dt,
        speed[4] * rx * dt};

    if (std::abs(delta[0]) >= min_step_pwm_)
    {
        target_pwms_[0] += delta[0];
    }
    if (std::abs(delta[1]) >= min_step_pwm_)
    {
        target_pwms_[1] += delta[1];
    }
    if (std::abs(delta[4]) >= min_step_pwm_)
    {
        target_pwms_[4] += delta[4];
    }

    const double ry_delta = speed[right_y_selected_servo_] * ry * dt;
    if (std::abs(ry_delta) >= min_step_pwm_)
    {
        target_pwms_[right_y_selected_servo_] += ry_delta;
    }

    bool changed = false;

    for (size_t i = 0; i < target_pwms_.size(); ++i)
    {
        target_pwms_[i] = std::clamp(target_pwms_[i], min_pwm_[i], max_pwm_[i]);
        target_joints_rad_[i] = pwmToRad(target_pwms_[i]);
        if (std::abs(target_pwms_[i] - old_pwms[i]) >= min_step_pwm_)
        {
            changed = true;
        }
    }

    return changed;
}
