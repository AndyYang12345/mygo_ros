#include "state_machine/vision_task_state.hpp"

#include <array>
#include <algorithm>
#include <cmath>

#include "example_interfaces/msg/float64_multi_array.hpp"
#include "state_machine/robot_state_machine_node.hpp"

namespace
{
constexpr std::array<double, 5> kVisionInitPwms = {
    1500.0, 1350.0, 2300.0, 1500.0, 1500.0};
constexpr int kButtonLB = 4;
constexpr int kButtonX = 2;
constexpr int kButtonRB = 5;
constexpr int kButtonB = 1;
constexpr uint8_t kPressEvent = 0;
constexpr uint8_t kReleaseEvent = 1;
constexpr float kSubmenuDeadzone = 0.25F;

const char *kSubmenuItems[8] = {
    "SMALL", "-", "BIG", "-", "-", "-", "-", "-",
};

int angleToOctant(float x, float y)
{
    const float norm_x = -x;
    const float norm_y = y;
    constexpr float kPi = 3.14159265358979323846F;
    float angle = std::atan2(norm_y, norm_x);
    if (angle < 0.0F) {
        angle += 2.0F * kPi;
    }
    const float sector = (2.0F * kPi) / 8.0F;
    int octant = static_cast<int>(std::floor((angle + sector * 0.5F) / sector));
    octant %= 8;
    return octant;
}

void publishDirectPwm(RobotStateMachineNode *context, const std::array<double, 5> &pwms)
{
    example_interfaces::msg::Float64MultiArray msg;
    msg.data.assign(pwms.begin(), pwms.end());
    context->getArmDirectPwmPub()->publish(msg);
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
    if (submenu_active_) {
        return static_cast<uint8_t>(std::clamp(submenu_selection_, 0, 7));
    }
    return static_cast<uint8_t>(energy_mode_ == EnergyMode::Big ? 1 : 0);
}

std::vector<std::string> VisionTaskState::getAvailableModes() const
{
    std::vector<std::string> names;
    names.reserve(8);
    for (const char *item : kSubmenuItems) {
        names.emplace_back(item);
    }
    return names;
}

void VisionTaskState::onEnter(RobotStateMachineNode *context)
{
    tracking_started_ = false;
    has_tracking_start_pwms_ = false;
    precision_mode_ = false;
    submenu_active_ = false;
    submenu_selection_ = 2;
    energy_mode_ = EnergyMode::Big;
    right_y_selected_servo_ = 2;
    dpad_switch_latched_ = false;
    target_pwms_ = kVisionInitPwms;
    tracking_start_pwms_ = kVisionInitPwms;

    refreshMainUi(context);

    publishDirectPwm(context, target_pwms_);
    RCLCPP_INFO(context->get_logger(), "Sent vision preset direct PWM target");

    auto query_current = std_msgs::msg::String();
    query_current.data = "kg";
    context->getArmQueryCurrentPub()->publish(query_current);

    auto camera_start = std_msgs::msg::String();
    camera_start.data = "start";
    context->getCameraStartAppPub()->publish(camera_start);

    RCLCPP_INFO(context->get_logger(), "Entered VISION_TASK state");
    RCLCPP_INFO(context->get_logger(), "Requested camera recognition START, waiting for A to enable tracking");
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
    if (msg->button_id == kButtonLB)
    {
        if (msg->event_type == kPressEvent) {
            submenu_active_ = true;
            submenu_selection_ = (energy_mode_ == EnergyMode::Big) ? 2 : 0;
            refreshSubmenuUi(context);
            RCLCPP_INFO(context->get_logger(), "VISION submenu opened, choose energy mode");
            return;
        }

        if (msg->event_type == kReleaseEvent && submenu_active_) {
            submenu_active_ = false;
            const bool changed = applyEnergyModeSelection(context);
            refreshMainUi(context);
            if (changed) {
                RCLCPP_INFO(context->get_logger(), "VISION energy mode applied via LB release");
            }
            return;
        }
    }

    if (submenu_active_ && msg->button_id == kButtonB && msg->event_type == kPressEvent) {
        submenu_active_ = false;
        refreshMainUi(context);
        RCLCPP_INFO(context->get_logger(), "VISION submenu canceled by B");
        return;
    }

    if (msg->event_type != kPressEvent)
    {
        return;
    }

    if (msg->button_id == 0)
    {
        auto query_current = std_msgs::msg::String();
        query_current.data = "kg";
        context->getArmQueryCurrentPub()->publish(query_current);

        std::array<double, 5> start_pwms = target_pwms_;
        if (context->getLatestArmCurrentPwm(start_pwms, 1.5)) {
            target_pwms_ = start_pwms;
            RCLCPP_INFO(
                context->get_logger(),
                "VISION start pose source=/arm/current_pwm yaw=%.1f pitch(servo3)=%.1f",
                start_pwms[0],
                start_pwms[3]);
        } else {
            RCLCPP_WARN(
                context->get_logger(),
                "VISION start pose fallback=local_cache yaw=%.1f pitch(servo3)=%.1f",
                start_pwms[0],
                start_pwms[3]);
        }

            tracking_start_pwms_ = start_pwms;
            has_tracking_start_pwms_ = true;

        auto camera_start = std_msgs::msg::String();
        camera_start.data = "yaw_pwm:" + std::to_string(static_cast<int>(std::lround(start_pwms[0]))) +
                            ",pitch_pwm:" + std::to_string(static_cast<int>(std::lround(start_pwms[3]))) +
                            ",tracking:1,sim_mode:" + energyModeToToken(energy_mode_);
        context->getCameraVisionStartPub()->publish(camera_start);
        tracking_started_ = true;
        RCLCPP_INFO(context->get_logger(), "Requested camera tracking START with init pose: %s", camera_start.data.c_str());
        return;
    }

    if (msg->button_id == kButtonX)
    {
        const std::array<double, 5> &restart_pwms = has_tracking_start_pwms_ ? tracking_start_pwms_ : target_pwms_;

        // X is a refresh command: force re-recognition from the saved pose.
        // Do not send stop first, otherwise stop/start on different topics can race and leave app in STOPPED.

        publishDirectPwm(context, restart_pwms);

        auto camera_start = std_msgs::msg::String();
        camera_start.data = "yaw_pwm:" + std::to_string(static_cast<int>(std::lround(restart_pwms[0]))) +
                    ",pitch_pwm:" + std::to_string(static_cast<int>(std::lround(restart_pwms[3]))) +
                    ",tracking:0,sim_mode:" + energyModeToToken(energy_mode_);
        context->getCameraVisionStartPub()->publish(camera_start);

        target_pwms_ = restart_pwms;
        tracking_start_pwms_ = restart_pwms;
        has_tracking_start_pwms_ = true;
        tracking_started_ = false;

        RCLCPP_INFO(
            context->get_logger(),
            "Requested camera re-identify from saved start pose: yaw=%.1f pitch(servo3)=%.1f",
            restart_pwms[0],
            restart_pwms[3]);
        return;
    }

    if (msg->button_id == 1)
    {
        auto camera_stop = std_msgs::msg::String();
        camera_stop.data = "stop";
        context->getCameraVisionStopPub()->publish(camera_stop);
        RCLCPP_INFO(context->get_logger(), "Requested camera vision task STOP");

        context->changeState(4);
        return;
    }

    if (msg->button_id == kButtonRB)
    {
        precision_mode_ = !precision_mode_;
        RCLCPP_INFO(
            context->get_logger(),
            "VISION manual mode switched to: %s",
            precision_mode_ ? "PRECISION" : "HIGH_SPEED");
    }
}

void VisionTaskState::handleJoystick(
    RobotStateMachineNode *context,
    const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
{
    if (submenu_active_ && (msg->joystick_id == 1 || msg->joystick_id == 2))
    {
        const float x = msg->x;
        const float y = msg->y;
        const float radius = std::sqrt(x * x + y * y);
        if (radius < kSubmenuDeadzone) {
            return;
        }

        const int octant = angleToOctant(x, y);
        if (octant != submenu_selection_) {
            submenu_selection_ = octant;
            refreshSubmenuUi(context);
        }
        return;
    }

    if (tracking_started_)
    {
        return;
    }

    const double deadzone = context->getJoystickDeadzone();
    const double speed_scale = precision_mode_ ? 12.0 : 24.0;

    const double lx = (std::abs(static_cast<double>(msg->x)) < deadzone) ? 0.0 : static_cast<double>(msg->x);
    const double ly = (std::abs(static_cast<double>(msg->y)) < deadzone) ? 0.0 : static_cast<double>(msg->y);

    bool changed = false;
    if (msg->joystick_id == 1)
    {
        const bool centered = std::abs(lx) < 1e-6 && std::abs(ly) < 1e-6;
        if (centered)
        {
            return;
        }

        const double delta_x = speed_scale * lx;
        const double delta_y = speed_scale * ly;
        if (std::abs(delta_x) >= 1.0)
        {
            target_pwms_[0] += delta_x;
            changed = true;
        }
        if (std::abs(delta_y) >= 1.0)
        {
            target_pwms_[1] += delta_y;
            changed = true;
        }
    }
    else if (msg->joystick_id == 2)
    {
        const double y = static_cast<double>(msg->y);
        const double threshold = 0.6;
        const double reset_threshold = 0.2;

        if (!dpad_switch_latched_ && y >= threshold)
        {
            right_y_selected_servo_ = 3;
            dpad_switch_latched_ = true;
            RCLCPP_INFO(context->get_logger(), "VISION right-stick-y target switched to servo 3");
        }
        else if (!dpad_switch_latched_ && y <= -threshold)
        {
            right_y_selected_servo_ = 2;
            dpad_switch_latched_ = true;
            RCLCPP_INFO(context->get_logger(), "VISION right-stick-y target switched to servo 2");
        }
        else if (std::abs(y) < reset_threshold)
        {
            dpad_switch_latched_ = false;
        }

        const double rx = (std::abs(static_cast<double>(msg->x)) < deadzone) ? 0.0 : static_cast<double>(msg->x);
        const double ry = (std::abs(static_cast<double>(msg->y)) < deadzone) ? 0.0 : static_cast<double>(msg->y);
        const double delta_rx = speed_scale * rx;
        const double delta_ry = speed_scale * ry;
        if (std::abs(delta_rx) >= 1.0)
        {
            target_pwms_[4] += delta_rx;
            changed = true;
        }
        if (std::abs(delta_ry) >= 1.0)
        {
            if (right_y_selected_servo_ == 2)
            {
                target_pwms_[right_y_selected_servo_] += delta_ry;
            }
            else
            {
                target_pwms_[right_y_selected_servo_] -= delta_ry;
            }
            changed = true;
        }
    }

    if (changed)
    {
        for (double &pwm : target_pwms_)
        {
            pwm = std::clamp(pwm, 500.0, 2500.0);
        }
        publishDirectPwm(context, target_pwms_);
    }
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

void VisionTaskState::refreshMainUi(RobotStateMachineNode *context)
{
    const std::string mode_text = "LB模式:" + std::string(energy_mode_ == EnergyMode::Big ? "大能量" : "小能量");
    context->setMenuItems({mode_text, "A开始追踪", "X重新识别并回到起始位", "B结束识别并返回菜单"});
    context->setMenuSelection(1);
}

void VisionTaskState::refreshSubmenuUi(RobotStateMachineNode *context)
{
    std::vector<std::string> items;
    items.reserve(8);
    for (const char *name : kSubmenuItems) {
        items.emplace_back(name);
    }
    context->setMenuItems(items);
    context->setMenuSelection(std::clamp(submenu_selection_, 0, 7));
}

std::string VisionTaskState::energyModeToToken(EnergyMode mode)
{
    return mode == EnergyMode::Big ? "big" : "small";
}

bool VisionTaskState::applyEnergyModeSelection(RobotStateMachineNode *context)
{
    EnergyMode new_mode = energy_mode_;
    if (submenu_selection_ == 0) {
        new_mode = EnergyMode::Small;
    } else if (submenu_selection_ == 2) {
        new_mode = EnergyMode::Big;
    } else {
        RCLCPP_INFO(context->get_logger(), "VISION submenu selected no-op slot, keep current mode");
        return false;
    }

    const bool changed = (new_mode != energy_mode_);
    energy_mode_ = new_mode;

    // Mode switch always triggers re-recognition and simulation regeneration.
    auto camera_stop = std_msgs::msg::String();
    camera_stop.data = "stop";
    context->getCameraVisionStopPub()->publish(camera_stop);

    const std::array<double, 5> &restart_pwms = has_tracking_start_pwms_ ? tracking_start_pwms_ : target_pwms_;
    publishDirectPwm(context, restart_pwms);

    auto camera_start = std_msgs::msg::String();
    camera_start.data = "yaw_pwm:" + std::to_string(static_cast<int>(std::lround(restart_pwms[0]))) +
                        ",pitch_pwm:" + std::to_string(static_cast<int>(std::lround(restart_pwms[3]))) +
                        ",tracking:0,sim_mode:" + energyModeToToken(energy_mode_);
    context->getCameraVisionStartPub()->publish(camera_start);

    target_pwms_ = restart_pwms;
    tracking_start_pwms_ = restart_pwms;
    has_tracking_start_pwms_ = true;
    tracking_started_ = false;

    RCLCPP_INFO(
        context->get_logger(),
        "VISION mode switched to %s, re-recognition started from yaw=%.1f pitch(servo3)=%.1f",
        energyModeToToken(energy_mode_).c_str(),
        restart_pwms[0],
        restart_pwms[3]);

    return changed;
}
