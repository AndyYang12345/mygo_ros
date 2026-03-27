#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <custom_interfaces/msg/robot_state.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_hardware_interfaces/serial_port.hpp"

namespace
{
constexpr size_t kTotalServoCount = 6;
constexpr char kVisionStateName[] = "VISION_TASK";

bool parse_fixed_digits(
    const std::string &text,
    size_t start,
    size_t width,
    int &value)
{
    if (start + width > text.size())
    {
        return false;
    }

    int parsed = 0;
    for (size_t i = 0; i < width; ++i)
    {
        const char ch = text[start + i];
        if (!std::isdigit(static_cast<unsigned char>(ch)))
        {
            return false;
        }
        parsed = parsed * 10 + (ch - '0');
    }
    value = parsed;
    return true;
}
}

class VisionBluetoothBridgeNode : public rclcpp::Node
{
public:
    VisionBluetoothBridgeNode()
    : Node("vision_bluetooth_bridge_node")
    {
        bluetooth_device_ = this->declare_parameter<std::string>("bluetooth_device", "/dev/rfcomm0");
        bluetooth_baudrate_ = this->declare_parameter<int>("bluetooth_baudrate", 115200);
        read_timeout_ms_ = this->declare_parameter<int>("read_timeout_ms", 20);
        reconnect_interval_ms_ = this->declare_parameter<int>("reconnect_interval_ms", 1000);
        forward_only_in_vision_task_ = this->declare_parameter<bool>("forward_only_in_vision_task", true);
        vision_state_name_ = this->declare_parameter<std::string>("vision_state_name", kVisionStateName);
        log_forwarded_frames_ = this->declare_parameter<bool>("log_forwarded_frames", false);

        const auto default_pwms = this->declare_parameter<std::vector<int64_t>>(
            "default_arm_pwms",
            std::vector<int64_t>{1500, 1350, 2300, 1500, 1500, 1500});
        load_default_pwms(default_pwms);

        direct_pwm_pub_ = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "/cmd/arm/direct_pwm_command",
            10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/status/vision/bluetooth_bridge",
            10);
        robot_state_sub_ = this->create_subscription<custom_interfaces::msg::RobotState>(
            "/robot/state",
            10,
            std::bind(&VisionBluetoothBridgeNode::on_robot_state, this, std::placeholders::_1));

        read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(std::max(5, read_timeout_ms_)),
            std::bind(&VisionBluetoothBridgeNode::poll_bluetooth_frame, this));

        publish_status("vision bluetooth bridge started, device=" + bluetooth_device_);
    }

private:
    struct VisionServoFrame
    {
        std::array<int, kTotalServoCount> pwm_values{};
        std::array<bool, kTotalServoCount> touched{};
        int duration_ms{0};
    };

    void load_default_pwms(const std::vector<int64_t> &defaults)
    {
        last_arm_pwms_ = {1500, 1350, 2300, 1500, 1500, 1500};
        for (size_t i = 0; i < std::min(defaults.size(), last_arm_pwms_.size()); ++i)
        {
            last_arm_pwms_[i] = static_cast<int>(std::clamp<int64_t>(defaults[i], 500, 2500));
        }
    }

    void on_robot_state(const custom_interfaces::msg::RobotState::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        const bool was_active = vision_task_active_;
        vision_task_active_ = (msg->state_name == vision_state_name_);
        if (was_active == vision_task_active_)
        {
            return;
        }

        const std::string state_text = vision_task_active_ ? "entered" : "left";
        RCLCPP_INFO(
            this->get_logger(),
            "Vision bridge %s vision task mode, state_name=%s",
            state_text.c_str(),
            msg->state_name.c_str());
    }

    void poll_bluetooth_frame()
    {
        if (!ensure_bluetooth_ready())
        {
            return;
        }

        std::string frame;
        if (!bluetooth_port_.read_braced_frame(frame, read_timeout_ms_))
        {
            return;
        }

        auto parsed = parse_vision_frame(frame);
        if (!parsed.has_value())
        {
            RCLCPP_WARN(this->get_logger(), "Discard invalid vision bluetooth frame: %s", frame.c_str());
            return;
        }

        if (forward_only_in_vision_task_ && !vision_task_active_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Received vision bluetooth frame outside %s, frame ignored.",
                vision_state_name_.c_str());
            return;
        }

        publish_direct_pwm(parsed.value(), frame);
    }

    bool ensure_bluetooth_ready()
    {
        if (bluetooth_port_.is_open())
        {
            return true;
        }

        const auto now = this->now();
        if ((now - last_connect_attempt_).nanoseconds() < static_cast<int64_t>(reconnect_interval_ms_) * 1000000LL)
        {
            return false;
        }
        last_connect_attempt_ = now;

        if (!bluetooth_port_.open(bluetooth_device_, bluetooth_baudrate_))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to open bluetooth serial device %s at %d baud",
                bluetooth_device_.c_str(),
                bluetooth_baudrate_);
            return false;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Bluetooth serial connected: %s @ %d",
            bluetooth_device_.c_str(),
            bluetooth_baudrate_);
        publish_status("bluetooth serial connected: " + bluetooth_device_);
        return true;
    }

    std::optional<VisionServoFrame> parse_vision_frame(const std::string &frame) const
    {
        if (frame.size() < 2 || frame.front() != '{' || frame.back() != '}')
        {
            return std::nullopt;
        }

        VisionServoFrame parsed{};
        size_t cursor = 1;
        bool has_any_servo = false;

        while (cursor + 1 < frame.size())
        {
            if (frame[cursor] != '#')
            {
                ++cursor;
                continue;
            }

            int servo_id = 0;
            int pwm = 0;
            int duration_ms = 0;
            if (!parse_fixed_digits(frame, cursor + 1, 3, servo_id))
            {
                return std::nullopt;
            }

            const size_t pwm_mark = cursor + 4;
            if (pwm_mark >= frame.size() || frame[pwm_mark] != 'P')
            {
                return std::nullopt;
            }
            if (!parse_fixed_digits(frame, pwm_mark + 1, 4, pwm))
            {
                return std::nullopt;
            }

            const size_t duration_mark = pwm_mark + 5;
            if (duration_mark >= frame.size() || frame[duration_mark] != 'T')
            {
                return std::nullopt;
            }
            if (!parse_fixed_digits(frame, duration_mark + 1, 4, duration_ms))
            {
                return std::nullopt;
            }

            const size_t end_mark = duration_mark + 5;
            if (end_mark >= frame.size() || frame[end_mark] != '!')
            {
                return std::nullopt;
            }

            if (servo_id >= 0 && servo_id < static_cast<int>(kTotalServoCount))
            {
                parsed.pwm_values[static_cast<size_t>(servo_id)] = std::clamp(pwm, 500, 2500);
                parsed.touched[static_cast<size_t>(servo_id)] = true;
                parsed.duration_ms = std::max(parsed.duration_ms, duration_ms);
                has_any_servo = true;
            }

            cursor = end_mark + 1;
        }

        if (!has_any_servo)
        {
            return std::nullopt;
        }

        return parsed;
    }

    void publish_direct_pwm(const VisionServoFrame &frame, const std::string &raw_frame)
    {
        for (size_t servo_id = 0; servo_id < kTotalServoCount; ++servo_id)
        {
            if (frame.touched[servo_id])
            {
                last_arm_pwms_[servo_id] = frame.pwm_values[servo_id];
            }
        }

        example_interfaces::msg::Float64MultiArray msg;
        msg.data.reserve(kTotalServoCount + 1);
        for (const int pwm : last_arm_pwms_)
        {
            msg.data.push_back(static_cast<double>(pwm));
        }
        msg.data.push_back(static_cast<double>(std::max(frame.duration_ms, 20)));
        direct_pwm_pub_->publish(msg);

        if (log_forwarded_frames_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Forwarded vision frame: raw=%s -> pwm=[%d,%d,%d,%d,%d,%d] duration=%d",
                raw_frame.c_str(),
                last_arm_pwms_[0],
                last_arm_pwms_[1],
                last_arm_pwms_[2],
                last_arm_pwms_[3],
                last_arm_pwms_[4],
                last_arm_pwms_[5],
                std::max(frame.duration_ms, 20));
        }
    }

    void publish_status(const std::string &text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        status_pub_->publish(msg);
    }

    std::string bluetooth_device_;
    int bluetooth_baudrate_{115200};
    int read_timeout_ms_{20};
    int reconnect_interval_ms_{1000};
    bool forward_only_in_vision_task_{true};
    bool vision_task_active_{false};
    bool log_forwarded_frames_{false};
    std::string vision_state_name_{kVisionStateName};

    rclcpp::Time last_connect_attempt_{0, 0, RCL_ROS_TIME};
    std::array<int, kTotalServoCount> last_arm_pwms_{};

    SerialPort bluetooth_port_;

    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr direct_pwm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<custom_interfaces::msg::RobotState>::SharedPtr robot_state_sub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionBluetoothBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
