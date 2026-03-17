#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "custom_interfaces/msg/pole_command.hpp"
#include "robot_hardware_interfaces/send_command.hpp"

using custom_interfaces::msg::PoleCommand;

class PoleSerialNode : public rclcpp::Node
{
public:
	PoleSerialNode()
	: Node("pole_serial_node")
	{
		const auto mode_name = this->declare_parameter<std::string>("mode_name", "POLE");
		sender_ = std::make_unique<SendCommand>(mode_name);

		if (!sender_->initialize()) {
			const auto & cfg = sender_->config();
			RCLCPP_ERROR(
				this->get_logger(),
				"Failed to open serial for mode=%s, device=%s, baud=%d",
				cfg.mode_name.c_str(),
				cfg.device.c_str(),
				cfg.baudrate);
		} else {
			const auto & cfg = sender_->config();
			RCLCPP_INFO(
				this->get_logger(),
				"Serial ready for mode=%s, device=%s, baud=%d",
				cfg.mode_name.c_str(),
				cfg.device.c_str(),
				cfg.baudrate);
		}

		rotate_sub_ = this->create_subscription<PoleCommand>(
			"/cmd/pole/rotate",
			10,
			std::bind(&PoleSerialNode::rotateCallback, this, std::placeholders::_1));
	}

private:
	static std::string formatPayload(bool id, float delta)
	{
		const float clamped = std::clamp(delta, -1.0F, 1.0F);
		const bool is_negative = std::signbit(clamped);

		std::ostringstream ss;
		ss << "#00" << (id ? '1' : '0') << (is_negative ? 'N' : 'P');
		ss << std::fixed << std::setprecision(2) << std::abs(clamped) << "!";
		return ss.str();
	}

	void rotateCallback(const PoleCommand::SharedPtr msg)
	{
		if (!msg) {
			return;
		}

		if (!sender_ || !sender_->is_ready()) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				2000,
				"Pole sender is not ready, command skipped");
			return;
		}

		const auto payload = formatPayload(msg->id, msg->delta);
		if (!sender_->send(payload)) {
			RCLCPP_ERROR_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				2000,
				"Failed to send pole payload: %s",
				payload.c_str());
			return;
		}

		RCLCPP_DEBUG(
			this->get_logger(),
			"Sent pole payload: %s",
			payload.c_str());
	}

	rclcpp::Subscription<PoleCommand>::SharedPtr rotate_sub_;
	std::unique_ptr<SendCommand> sender_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PoleSerialNode>());
	rclcpp::shutdown();
	return 0;
}
