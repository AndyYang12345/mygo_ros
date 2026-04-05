#include "state_machine/arm_state.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

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
constexpr const char *kEmptyPresetSlot = "-";
constexpr const char *kHomePresetName = "home";
constexpr size_t kArmJointCount = 5;
const char *kOctantNames[8] = {
    "RIGHT", "UP_RIGHT", "UP", "UP_LEFT", "LEFT", "DOWN_LEFT", "DOWN", "DOWN_RIGHT"};
const std::array<const char *, kArmJointCount> kArmJointNames = {
    "joint1", "joint2", "joint3", "joint4", "joint5"};

double pwmToRad(double pwm)
{
    const double degree = (pwm - 1500.0) / 1000.0 * 135.0;
    return degree * static_cast<double>(kPi) / 180.0;
}

double radToDeg(double rad)
{
    return rad * 180.0 / static_cast<double>(kPi);
}

std::filesystem::path poseSnapshotDir()
{
    const auto source_path = std::filesystem::path(__FILE__);
    return source_path.parent_path().parent_path().parent_path() / "debug" / "arm_pose_snapshots";
}

std::pair<std::string, std::string> makeSnapshotTimestamps()
{
    const auto now = std::chrono::system_clock::now();
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_tm{};
    localtime_r(&now_time, &local_tm);

    std::ostringstream file_stamp;
    file_stamp << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
               << '_' << std::setw(3) << std::setfill('0') << millis.count();

    std::ostringstream display_stamp;
    display_stamp << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                  << '.' << std::setw(3) << std::setfill('0') << millis.count();

    return {file_stamp.str(), display_stamp.str()};
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
    // Default selection to HOME to avoid accidental execution of the previous target.
    submenu_selection_ = homePresetIndex();
    precision_mode_ = false;
    right_y_selected_servo_ = 2;
    dpad_switch_latched_ = false;
    rt_press_latched_ = false;
    target_joints_initialized_ = false;
    waiting_initial_state_ = true;
    preset_sync_pending_ = false;
    preset_motion_in_progress_ = false;
    preset_feedback_gate_ = false;
    preset_feedback_query_sent_ = false;
    preset_pre_sync_pending_ = false;
    post_preset_sync_armed_ = false;
    waiting_post_preset_feedback_ = false;
    skip_direct_target_refresh_once_ = false;
    rebase_on_next_manual_cycle_ = false;
    manual_reacquire_required_ = false;
    require_stick_center_rebase_ = false;
    manual_session_active_ = false;
    named_target_release_pending_ = false;
    pending_named_target_.clear();
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

                if (preset_feedback_gate_ && !preset_feedback_query_sent_)
                {
                    return;
                }

                for (size_t i = 0; i < target_pwms_.size(); ++i)
                {
                    latest_feedback_pwms_[i] = msg->data[i];
                    latest_feedback_joints_rad_[i] = pwmToRad(msg->data[i]);
                }
                latest_feedback_valid_ = true;
                latest_feedback_time_ = context->now();

                const bool should_update_direct_target =
                    ((!target_joints_initialized_) || waiting_initial_state_ || preset_sync_pending_) &&
                    !skip_direct_target_refresh_once_;
                const bool force_post_preset_refresh =
                    post_preset_sync_armed_ && !skip_direct_target_refresh_once_;

                if (should_update_direct_target || force_post_preset_refresh)
                {
                    for (size_t i = 0; i < target_pwms_.size(); ++i)
                    {
                        target_pwms_[i] = msg->data[i];
                        target_joints_rad_[i] = latest_feedback_joints_rad_[i];
                    }
                    target_joints_initialized_ = true;
                    waiting_initial_state_ = false;
                    if (post_preset_sync_armed_)
                    {
                        post_preset_sync_armed_ = false;
                        waiting_post_preset_feedback_ = false;
                        preset_motion_in_progress_ = false;
                        manual_reacquire_required_ = false;
                        rebase_on_next_manual_cycle_ = false;
                        require_stick_center_rebase_ = true;
                        manual_session_active_ = false;
                        RCLCPP_INFO(
                            context->get_logger(),
                            "ARM post-preset kg sync applied; waiting stick-center to start manual tuning");
                    }
                }

                if (kg_sync_requested_)
                {
                    kg_sync_requested_ = false;
                }

                if (preset_feedback_gate_)
                {
                    preset_feedback_gate_ = false;
                    preset_feedback_query_sent_ = false;
                }

                if (skip_direct_target_refresh_once_)
                {
                    skip_direct_target_refresh_once_ = false;
                }

                if (preset_pre_sync_pending_ && !pending_named_target_.empty())
                {
                    preset_pre_sync_pending_ = false;
                    named_target_release_pending_ = true;
                    named_target_release_time_ =
                        context->now() + rclcpp::Duration::from_seconds(named_target_release_delay_s_);

                    RCLCPP_INFO(
                        context->get_logger(),
                        "ARM preset pre-sync done, delay %.0f ms then execute named target: %s",
                        named_target_release_delay_s_ * 1000.0,
                        pending_named_target_.c_str());
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
            std::string chosen = kHomePresetName;
            if (submenu_selection_ >= 0 && submenu_selection_ < static_cast<int>(presets_.size()))
            {
                chosen = presets_[submenu_selection_];
            }

            // "-" is an explicit empty slot: do not publish any target.
            if (!chosen.empty() && chosen != kEmptyPresetSlot)
            {
                target_joints_initialized_ = false;
                waiting_initial_state_ = true;
                preset_sync_pending_ = false;
                preset_motion_in_progress_ = true;
                preset_feedback_gate_ = true;
                preset_feedback_query_sent_ = true;
                preset_pre_sync_pending_ = true;
                skip_direct_target_refresh_once_ = true;
                named_target_release_pending_ = false;
                pending_named_target_ = chosen;

                auto query_msg = std_msgs::msg::String();
                query_msg.data = "kg";
                context->getArmQueryCurrentPub()->publish(query_msg);
                last_query_time_ = context->now();

                RCLCPP_INFO(
                    context->get_logger(),
                    "ARM submenu selected index %d -> request kg pre-sync before named target: %s",
                    submenu_selection_,
                    chosen.c_str());
            }
            else
            {
                RCLCPP_INFO(
                    context->get_logger(),
                    "ARM submenu selected index %d is empty ('-'), skipped execution",
                    submenu_selection_);
            }

            // Always reset selection back to HOME after closing submenu to avoid accidental re-trigger.
            submenu_selection_ = homePresetIndex();
            updateSubmenuUi(context);
        }
        return;
    }

    if (msg->button_id == kButtonLB)
    {
        submenu_active_ = true;
        // Highlight HOME when opening submenu; user can then deliberately choose another slot.
        submenu_selection_ = homePresetIndex();
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
        savePoseSnapshot(context);
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

    if (named_target_release_pending_)
    {
        if (context->now() < named_target_release_time_)
        {
            return;
        }

        const std::string exec_target = pending_named_target_;
        auto target = custom_interfaces::msg::ArmNamedTarget();
        target.target_name = exec_target;
        context->getArmNamedTargetPub()->publish(target);

        named_target_release_pending_ = false;
        pending_named_target_.clear();
        post_preset_sync_armed_ = false;
        preset_sync_pending_ = true;
        preset_motion_in_progress_ = true;
        waiting_post_preset_feedback_ = true;
        manual_reacquire_required_ = true;
        preset_feedback_gate_ = true;
        preset_feedback_query_sent_ = false;
        preset_sync_due_time_ = context->now() + rclcpp::Duration::from_seconds(preset_sync_delay_s_);

        RCLCPP_INFO(
            context->get_logger(),
            "ARM execute delayed named target after kg pre-sync");
        return;
    }

    if (preset_motion_in_progress_ && context->now() < preset_sync_due_time_)
    {
        return;
    }

    const auto now = context->now();
    if (preset_sync_pending_ && now >= preset_sync_due_time_)
    {
        auto query_msg = std_msgs::msg::String();
        query_msg.data = "kg";
        context->getArmQueryCurrentPub()->publish(query_msg);
        last_query_time_ = now;
        preset_feedback_query_sent_ = true;
        preset_sync_pending_ = false;
        post_preset_sync_armed_ = true;
        if (waiting_post_preset_feedback_)
        {
            preset_motion_in_progress_ = true;
            RCLCPP_INFO(
                context->get_logger(),
                "ARM post-preset kg requested, waiting feedback before enabling joystick");
        }
        return;
    }

    if (waiting_post_preset_feedback_)
    {
        RCLCPP_WARN_THROTTLE(
            context->get_logger(),
            *context->get_clock(),
            1000,
            "ARM waiting post-preset kg feedback, joystick control still locked.");
        return;
    }

    if (!target_joints_initialized_)
    {
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
        preset_feedback_query_sent_ = true;
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

int ArmState::homePresetIndex() const
{
    for (size_t i = 0; i < presets_.size(); ++i)
    {
        if (presets_[i] == kHomePresetName)
        {
            return static_cast<int>(i);
        }
    }
    return 0;
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

void ArmState::savePoseSnapshot(RobotStateMachineNode *context)
{
    if (!latest_feedback_valid_)
    {
        auto query_msg = std_msgs::msg::String();
        query_msg.data = "kg";
        context->getArmQueryCurrentPub()->publish(query_msg);
        last_query_time_ = context->now();

        RCLCPP_WARN(
            context->get_logger(),
            "ARM pose snapshot skipped: no current hardware feedback yet, requested kg refresh.");
        return;
    }

    const auto [file_stamp, display_stamp] = makeSnapshotTimestamps();
    const auto output_dir = poseSnapshotDir();
    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec)
    {
        RCLCPP_ERROR(
            context->get_logger(),
            "Failed to create arm pose snapshot dir %s: %s",
            output_dir.c_str(),
            ec.message().c_str());
        return;
    }

    const auto output_path = output_dir / ("arm_pose_" + file_stamp + ".md");
    std::ofstream out(output_path);
    if (!out.is_open())
    {
        RCLCPP_ERROR(
            context->get_logger(),
            "Failed to open arm pose snapshot file: %s",
            output_path.c_str());
        return;
    }

    out << "# Arm Pose Snapshot\n\n";
    out << "- Timestamp: " << display_stamp << "\n";
    out << "- Source: `/arm/current_pwm` feedback converted to angles\n";
    out << "- State: ARM\n";
    out << "- Precision Mode: " << (precision_mode_ ? "ON" : "OFF") << "\n";
    out << "- Right Stick Y Servo: " << right_y_selected_servo_ << "\n\n";

    out << "## Joint Table\n\n";
    out << "| Servo ID | Joint | Angle (deg) | Angle (rad) |\n";
    out << "| --- | --- | ---: | ---: |\n";
    for (size_t i = 0; i < latest_feedback_joints_rad_.size(); ++i)
    {
        out << "| " << i
            << " | " << kArmJointNames[i]
            << " | " << std::fixed << std::setprecision(2) << radToDeg(latest_feedback_joints_rad_[i])
            << " | " << std::fixed << std::setprecision(4) << latest_feedback_joints_rad_[i]
            << " |\n";
    }

    out << "\n## MoveIt2 Copy Block\n\n";
    out << "```yaml\n";
    for (size_t i = 0; i < latest_feedback_joints_rad_.size(); ++i)
    {
        out << kArmJointNames[i] << ": "
            << std::fixed << std::setprecision(4) << latest_feedback_joints_rad_[i] << '\n';
    }
    out << "```\n\n";

    out << "## Joint Vector\n\n";
    out << "```text\n[";
    for (size_t i = 0; i < latest_feedback_joints_rad_.size(); ++i)
    {
        if (i > 0)
        {
            out << ", ";
        }
        out << std::fixed << std::setprecision(4) << latest_feedback_joints_rad_[i];
    }
    out << "]\n```\n";
    out.close();

    RCLCPP_INFO(
        context->get_logger(),
        "ARM pose snapshot saved: %s",
        output_path.c_str());
}

bool ArmState::applyAxisControl(RobotStateMachineNode *context, double dt)
{
    const auto &left = context->getLeftJoystick();
    const auto &right = context->getRightJoystick();
    const double deadzone = context->getJoystickDeadzone();

    const bool left_centered_now =
        (std::abs(static_cast<double>(left.x)) < deadzone) &&
        (std::abs(static_cast<double>(left.y)) < deadzone);
    const bool right_centered_now =
        (std::abs(static_cast<double>(right.x)) < deadzone) &&
        (std::abs(static_cast<double>(right.y)) < deadzone);
    const bool sticks_centered_now = left_centered_now && right_centered_now;

    if (sticks_centered_now)
    {
        manual_session_active_ = false;
    }

    if (manual_reacquire_required_)
    {
        const auto now = context->now();
        const bool feedback_fresh =
            latest_feedback_valid_ && ((now - latest_feedback_time_).seconds() <= 0.35);

        if (!feedback_fresh)
        {
            if ((now - last_query_time_).seconds() > 0.20)
            {
                auto query_msg = std_msgs::msg::String();
                query_msg.data = "kg";
                context->getArmQueryCurrentPub()->publish(query_msg);
                last_query_time_ = now;
            }
            return false;
        }

        for (size_t i = 0; i < target_pwms_.size(); ++i)
        {
            target_pwms_[i] = latest_feedback_pwms_[i];
            target_joints_rad_[i] = latest_feedback_joints_rad_[i];
        }
        manual_reacquire_required_ = false;
        rebase_on_next_manual_cycle_ = false;

        RCLCPP_INFO(
            context->get_logger(),
            "ARM manual baseline reset to latest kg pose before joystick control");
    }

    if (require_stick_center_rebase_)
    {
        if (latest_feedback_valid_)
        {
            for (size_t i = 0; i < target_pwms_.size(); ++i)
            {
                target_pwms_[i] = latest_feedback_pwms_[i];
                target_joints_rad_[i] = latest_feedback_joints_rad_[i];
            }
        }

        if (!sticks_centered_now)
        {
            return false;
        }

        require_stick_center_rebase_ = false;
        RCLCPP_INFO(
            context->get_logger(),
            "ARM joystick centered, manual tuning starts from latest kg pose");
    }

    if (rebase_on_next_manual_cycle_ && latest_feedback_valid_)
    {
        for (size_t i = 0; i < target_pwms_.size(); ++i)
        {
            target_pwms_[i] = latest_feedback_pwms_[i];
            target_joints_rad_[i] = latest_feedback_joints_rad_[i];
        }
        rebase_on_next_manual_cycle_ = false;
    }

    const double lx = (std::abs(left.x) < deadzone) ? 0.0 : static_cast<double>(left.x);
    const double ly = (std::abs(left.y) < deadzone) ? 0.0 : static_cast<double>(left.y);
    const double rx = (std::abs(right.x) < deadzone) ? 0.0 : static_cast<double>(right.x);
    const double ry = (std::abs(right.y) < deadzone) ? 0.0 : static_cast<double>(right.y);

    const bool has_manual_input =
        (std::abs(lx) > 0.0) || (std::abs(ly) > 0.0) || (std::abs(rx) > 0.0) || (std::abs(ry) > 0.0);

    if (has_manual_input && !manual_session_active_)
    {
        const auto now = context->now();
        const bool feedback_fresh =
            latest_feedback_valid_ && ((now - latest_feedback_time_).seconds() <= 0.35);

        if (!feedback_fresh)
        {
            if ((now - last_query_time_).seconds() > 0.20)
            {
                auto query_msg = std_msgs::msg::String();
                query_msg.data = "kg";
                context->getArmQueryCurrentPub()->publish(query_msg);
                last_query_time_ = now;
            }
            return false;
        }

        for (size_t i = 0; i < target_pwms_.size(); ++i)
        {
            target_pwms_[i] = latest_feedback_pwms_[i];
            target_joints_rad_[i] = latest_feedback_joints_rad_[i];
        }
        manual_session_active_ = true;

        RCLCPP_INFO(
            context->get_logger(),
            "ARM manual session started from fresh kg baseline");
    }

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
        if (right_y_selected_servo_ == 2){
            target_pwms_[right_y_selected_servo_] += ry_delta;
        }else {
            target_pwms_[right_y_selected_servo_] -= ry_delta;
        }
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
