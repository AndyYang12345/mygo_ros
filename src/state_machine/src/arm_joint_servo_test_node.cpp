#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "custom_interfaces/msg/arm_joint_target.hpp"
#include "custom_interfaces/msg/button_intent.hpp"
#include "custom_interfaces/msg/joystick_intent.hpp"

class ArmJointServoTestNode : public rclcpp::Node
{
public:
    ArmJointServoTestNode()
    : Node("arm_joint_servo_test_node")
    {
        servo_count_ = this->declare_parameter<int>("servo_count", 5);
        step_rad_ = this->declare_parameter<double>("step_rad", 0.02);
        deadzone_ = this->declare_parameter<double>("deadzone", 0.15);

        if (servo_count_ <= 0)
        {
            servo_count_ = 5;
        }

        const double center_deg = 135.0;
        const double center_rad = degreeToRad(center_deg);
        joints_rad_.assign(static_cast<size_t>(servo_count_), center_rad);

        arm_joint_pub_ = this->create_publisher<custom_interfaces::msg::ArmJointTarget>(
            "/cmd/arm/joint_target", 10);

        button_sub_ = this->create_subscription<custom_interfaces::msg::ButtonIntent>(
            "button_intent", 10,
            std::bind(&ArmJointServoTestNode::onButtonIntent, this, std::placeholders::_1));

        joystick_sub_ = this->create_subscription<custom_interfaces::msg::JoystickIntent>(
            "joystick_intent", 10,
            std::bind(&ArmJointServoTestNode::onJoystickIntent, this, std::placeholders::_1));

        publishCurrentJoints("startup center (equivalent P1500)");
        RCLCPP_INFO(
            this->get_logger(),
            "Arm joint servo test started: servo_count=%d, selected_servo=%d, step_rad=%.4f",
            servo_count_, selected_servo_id_, step_rad_);
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr int kButtonLB = 4;
    static constexpr int kButtonRB = 5;
    static constexpr uint8_t kPressEvent = 0;

    static double radToDegree(double rad)
    {
        return rad * 180.0 / kPi;
    }

    static double degreeToRad(double degree)
    {
        return degree * kPi / 180.0;
    }

    static double clamp(double value, double min_v, double max_v)
    {
        return std::max(min_v, std::min(max_v, value));
    }

    void publishCurrentJoints(const std::string &reason)
    {
        custom_interfaces::msg::ArmJointTarget cmd;
        cmd.joints = joints_rad_;
        arm_joint_pub_->publish(cmd);

        RCLCPP_INFO(
            this->get_logger(),
            "Published ArmJointTarget (%s): selected_servo=%d, angle_deg=%.2f",
            reason.c_str(),
            selected_servo_id_,
            radToDegree(joints_rad_[static_cast<size_t>(selected_servo_id_)]));
    }

    void onButtonIntent(const custom_interfaces::msg::ButtonIntent::SharedPtr msg)
    {
        if (msg->event_type != kPressEvent)
        {
            return;
        }

        if (msg->button_id == kButtonRB)
        {
            selected_servo_id_ = (selected_servo_id_ + 1) % servo_count_;
            RCLCPP_INFO(this->get_logger(), "Selected servo -> %d", selected_servo_id_);
            return;
        }

        if (msg->button_id == kButtonLB)
        {
            selected_servo_id_ = (selected_servo_id_ - 1 + servo_count_) % servo_count_;
            RCLCPP_INFO(this->get_logger(), "Selected servo -> %d", selected_servo_id_);
        }
    }

    void onJoystickIntent(const custom_interfaces::msg::JoystickIntent::SharedPtr msg)
    {
        if (msg->joystick_id != 0)
        {
            return;
        }

        const double y = static_cast<double>(msg->y);
        if (std::abs(y) < deadzone_)
        {
            return;
        }

        const double delta = y * step_rad_;
        auto &joint = joints_rad_[static_cast<size_t>(selected_servo_id_)];
        joint = clamp(joint + delta, degreeToRad(0.0), degreeToRad(270.0));
        publishCurrentJoints("left joystick y step");
    }

    int servo_count_ = 5;
    int selected_servo_id_ = 0;
    double step_rad_ = 0.02;
    double deadzone_ = 0.15;

    std::vector<double> joints_rad_;

    rclcpp::Publisher<custom_interfaces::msg::ArmJointTarget>::SharedPtr arm_joint_pub_;
    rclcpp::Subscription<custom_interfaces::msg::ButtonIntent>::SharedPtr button_sub_;
    rclcpp::Subscription<custom_interfaces::msg::JoystickIntent>::SharedPtr joystick_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmJointServoTestNode>());
    rclcpp::shutdown();
    return 0;
}
