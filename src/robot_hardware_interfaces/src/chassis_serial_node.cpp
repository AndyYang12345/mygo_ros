#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_hardware_interfaces/send_command.hpp"

using std::placeholders::_1;
using geometry_msgs::msg::Twist;

class ChassisSerialNode : public rclcpp::Node
{
public:
    ChassisSerialNode()
        : Node("chassis_serial_node")
    {
        const auto mode_name = this->declare_parameter<std::string>("mode_name", "CHASSIS");
        sender_ = std::make_unique<SendCommand>(mode_name);

        if (!sender_->initialize())
        {
            const auto &cfg = sender_->config();
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to open serial for mode=%s, device=%s, baud=%d",
                cfg.mode_name.c_str(),
                cfg.device.c_str(),
                cfg.baudrate);
        }
        else
        {
            const auto &cfg = sender_->config();
            RCLCPP_INFO(
                this->get_logger(),
                "Serial ready for mode=%s, device=%s, baud=%d",
                cfg.mode_name.c_str(),
                cfg.device.c_str(),
                cfg.baudrate);
        }

        twist_sub_ = this->create_subscription<Twist>(
            "/cmd/chassis/velocity",
            10,
            std::bind(&ChassisSerialNode::chassis_cmd_callback, this, _1));
    }

private:
    void chassis_cmd_callback(const Twist::SharedPtr msg)
    {
        const double joy_linear = msg->linear.x;
        const double joy_angular = msg->angular.z;

        double left_output = joy_linear - joy_angular;
        double right_output = joy_linear + joy_angular;

        left_output = clamp(left_output, -1.0, 1.0);
        right_output = clamp(right_output, -1.0, 1.0);

        send_to_chassis_driver(left_output, right_output);
        RCLCPP_INFO(this->get_logger(), "Processed chassis cmd: left=%.3f, right=%.3f", left_output, right_output);
    }

    static double clamp(double value, double min_value, double max_value)
    {
        return std::max(min_value, std::min(max_value, value));
    }

    void send_to_chassis_driver(double left_speed, double right_speed)
    {
        if (!sender_ || !sender_->is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Chassis sender is not ready, command skipped");
            return;
        }

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "#" << left_speed << "," << right_speed << "\n";

        const auto payload = ss.str();
        if (!sender_->send(payload))
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to send chassis command over serial");
            return;
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "Sent chassis cmd: left=%.3f, right=%.3f, payload=%s",
            left_speed,
            right_speed,
            payload.c_str());
    }

    rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
    std::unique_ptr<SendCommand> sender_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisSerialNode>());
    rclcpp::shutdown();
    return 0;
}
