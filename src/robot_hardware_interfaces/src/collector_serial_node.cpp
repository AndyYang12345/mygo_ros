#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_hardware_interfaces/send_command.hpp"

using namespace std::chrono_literals;

class CollectorSerialNode : public rclcpp::Node
{
public:
    CollectorSerialNode()
        : Node("collector_serial_node")
    {
        const auto mode_name = this->declare_parameter<std::string>("mode_name", "COLLECTOR");
        op_cl_interval_ms_ = this->declare_parameter<int>("op_cl_interval_ms", 1000);

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

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/collector/ball_submission",
            10,
            std::bind(&CollectorSerialNode::commandCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "collector_serial_node started.");
    }

private:
    bool sendCommand(const std::string &cmd)
    {
        if (!sender_ || !sender_->is_ready())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Collector sender is not ready, command skipped");
            return false;
        }

        if (!sender_->send(cmd))
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to send collector command: %s",
                cmd.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Collector command sent: %s", cmd.c_str());
        return true;
    }

    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        if (waiting_auto_cl_)
        {
            RCLCPP_WARN(this->get_logger(), "OP->CL auto sequence running, ignore extra trigger");
            return;
        }

        if (trigger_step_ == 0)
        {
            if (sendCommand("UP"))
            {
                trigger_step_ = 1;
            }
            return;
        }

        if (trigger_step_ == 1)
        {
            if (!sendCommand("OP"))
            {
                return;
            }
            waiting_auto_cl_ = true;
            auto_cl_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(op_cl_interval_ms_),
                std::bind(&CollectorSerialNode::autoCloseCallback, this));
            trigger_step_ = 2;
            return;
        }

        if (trigger_step_ == 2)
        {
            if (sendCommand("DN"))
            {
                trigger_step_ = 0;
            }
        }
    }

    void autoCloseCallback()
    {
        if (auto_cl_timer_)
        {
            auto_cl_timer_->cancel();
        }
        if (!waiting_auto_cl_)
        {
            return;
        }

        if (!sendCommand("CL"))
        {
            RCLCPP_WARN(this->get_logger(), "CL auto send failed, waiting next manual trigger");
        }
        waiting_auto_cl_ = false;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr auto_cl_timer_;
    std::unique_ptr<SendCommand> sender_;

    int op_cl_interval_ms_ = 1000;
    int trigger_step_ = 0;
    bool waiting_auto_cl_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CollectorSerialNode>());
    rclcpp::shutdown();
    return 0;
}
