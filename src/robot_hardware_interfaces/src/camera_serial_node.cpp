#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

namespace
{
constexpr uint8_t kProtocolVersion = 0x01;
constexpr uint8_t kFlagIsResp = 0x80;
constexpr uint8_t kFlagRespOk = 0x40;
constexpr uint8_t kFlagIsReport = 0x20;

constexpr uint8_t CMD_SET_REPORT = 0xF8;
constexpr uint8_t CMD_APP_LIST = 0xF9;
constexpr uint8_t CMD_START_APP = 0xFA;
constexpr uint8_t CMD_EXIT_APP = 0xFB;
constexpr uint8_t CMD_CUR_APP_INFO = 0xFC;
constexpr uint8_t CMD_APP_INFO = 0xFD;
constexpr uint8_t CMD_KEY = 0xFE;
constexpr uint8_t CMD_TOUCH = 0xFF;

constexpr uint8_t CMD_CAMERA_SNAP = 0x01;

constexpr uint32_t kHeader = 0xBBACCAAA;
}

struct ProtocolFrame
{
	uint8_t flags{0};
	uint8_t cmd{0};
	std::vector<uint8_t> body;
};

class CameraTcpNode : public rclcpp::Node
{
public:
	CameraTcpNode()
	: Node("camera_serial_node")
	{
		camera_host_ = this->declare_parameter<std::string>("camera_host", "192.168.43.19");
		camera_port_ = this->declare_parameter<int>("camera_port", 5555);
		recv_timeout_ms_ = this->declare_parameter<int>("recv_timeout_ms", 3000);
		exit_confirm_word_ = this->declare_parameter<std::string>("exit_confirm_word", "EXIT_NOW");

		status_pub_ = this->create_publisher<std_msgs::msg::String>("/status/camera/protocol", 10);

		start_sub_ = this->create_subscription<std_msgs::msg::String>(
			"/cmd/camera/start_app",
			10,
			std::bind(&CameraTcpNode::on_start_app, this, std::placeholders::_1));

		exit_sub_ = this->create_subscription<std_msgs::msg::String>(
			"/cmd/camera/exit_app",
			10,
			std::bind(&CameraTcpNode::on_exit_app, this, std::placeholders::_1));

		query_sub_ = this->create_subscription<std_msgs::msg::String>(
			"/cmd/camera/query",
			10,
			std::bind(&CameraTcpNode::on_query, this, std::placeholders::_1));

		snap_sub_ = this->create_subscription<std_msgs::msg::Empty>(
			"/cmd/camera/snap",
			10,
			std::bind(&CameraTcpNode::on_snap, this, std::placeholders::_1));

		if (connect_if_needed()) {
			publish_status("camera tcp connected: " + camera_host_ + ":" + std::to_string(camera_port_));
		}
	}

	~CameraTcpNode() override
	{
		close_socket();
	}

private:
	static uint16_t crc16_ibm(const uint8_t *data, size_t size)
	{
		uint16_t crc = 0x0000;
		for (size_t i = 0; i < size; ++i) {
			crc ^= data[i];
			for (int bit = 0; bit < 8; ++bit) {
				if (crc & 0x01) {
					crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
				} else {
					crc = static_cast<uint16_t>(crc >> 1);
				}
			}
		}
		return crc;
	}

	static void append_u32_le(std::vector<uint8_t> &out, uint32_t value)
	{
		out.push_back(static_cast<uint8_t>(value & 0xFF));
		out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
		out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
		out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
	}

	static uint32_t read_u32_le(const uint8_t *ptr)
	{
		return static_cast<uint32_t>(ptr[0]) |
					 (static_cast<uint32_t>(ptr[1]) << 8) |
					 (static_cast<uint32_t>(ptr[2]) << 16) |
					 (static_cast<uint32_t>(ptr[3]) << 24);
	}

	static uint16_t read_u16_le(const uint8_t *ptr)
	{
		return static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8);
	}

	static std::string bytes_to_hex(const std::vector<uint8_t> &bytes)
	{
		std::ostringstream ss;
		ss << std::hex;
		for (size_t i = 0; i < bytes.size(); ++i) {
			if (i > 0) {
				ss << " ";
			}
			ss.width(2);
			ss.fill('0');
			ss << static_cast<int>(bytes[i]);
		}
		return ss.str();
	}

	static std::string trim(const std::string &value)
	{
		size_t left = 0;
		while (left < value.size() && std::isspace(static_cast<unsigned char>(value[left]))) {
			++left;
		}
		size_t right = value.size();
		while (right > left && std::isspace(static_cast<unsigned char>(value[right - 1]))) {
			--right;
		}
		return value.substr(left, right - left);
	}

	bool connect_if_needed()
	{
		if (socket_fd_ >= 0) {
			return true;
		}

		socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
		if (socket_fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to create tcp socket");
			return false;
		}

		sockaddr_in server_addr{};
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(static_cast<uint16_t>(camera_port_));
		if (::inet_pton(AF_INET, camera_host_.c_str(), &server_addr.sin_addr) <= 0) {
			RCLCPP_ERROR(this->get_logger(), "Invalid camera_host: %s", camera_host_.c_str());
			close_socket();
			return false;
		}

		if (::connect(socket_fd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
			RCLCPP_ERROR(
				this->get_logger(),
				"TCP connect failed: %s:%d",
				camera_host_.c_str(),
				camera_port_);
			close_socket();
			return false;
		}

		timeval timeout{};
		timeout.tv_sec = recv_timeout_ms_ / 1000;
		timeout.tv_usec = (recv_timeout_ms_ % 1000) * 1000;
		if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
			RCLCPP_WARN(this->get_logger(), "Failed to set recv timeout, continue with default socket timeout");
		}

		return true;
	}

	void close_socket()
	{
		if (socket_fd_ >= 0) {
			::close(socket_fd_);
			socket_fd_ = -1;
		}
	}

	bool recv_exact(std::vector<uint8_t> &out, size_t size)
	{
		out.clear();
		out.resize(size, 0);
		size_t total = 0;
		while (total < size) {
			const auto n = ::recv(socket_fd_, out.data() + total, size - total, 0);
			if (n <= 0) {
				if (n == 0) {
					RCLCPP_WARN(this->get_logger(), "Peer closed socket while receiving, want=%zu got=%zu", size, total);
				} else {
					RCLCPP_WARN(
						this->get_logger(),
						"recv failed while receiving, want=%zu got=%zu errno=%d(%s)",
						size,
						total,
						errno,
						std::strerror(errno));
				}
				return false;
			}
			total += static_cast<size_t>(n);
		}
		return true;
	}

	std::vector<uint8_t> build_frame(uint8_t cmd, const std::vector<uint8_t> &body)
	{
		std::vector<uint8_t> data;
		data.reserve(4 + 4 + 1 + 1 + body.size() + 2);

		append_u32_le(data, kHeader);

		const uint32_t data_len = static_cast<uint32_t>(1 + 1 + body.size() + 2);
		append_u32_le(data, data_len);

		const uint8_t flags = kProtocolVersion;
		data.push_back(flags);
		data.push_back(cmd);
		data.insert(data.end(), body.begin(), body.end());

		const uint16_t crc = crc16_ibm(data.data(), data.size());
		data.push_back(static_cast<uint8_t>(crc & 0xFF));
		data.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
		return data;
	}

	bool recv_frame(ProtocolFrame &frame)
	{
		std::vector<uint8_t> head;
		if (!recv_exact(head, 8)) {
			return false;
		}

		const uint32_t header = read_u32_le(head.data());
		if (header != kHeader) {
			RCLCPP_ERROR(this->get_logger(), "Invalid frame header: 0x%08X", header);
			return false;
		}

		const uint32_t data_len = read_u32_le(head.data() + 4);
		if (data_len < 4) {
			RCLCPP_ERROR(this->get_logger(), "Invalid data_len: %u", data_len);
			return false;
		}

		std::vector<uint8_t> payload;
		if (!recv_exact(payload, data_len)) {
			return false;
		}

		const auto recv_crc = read_u16_le(&payload[data_len - 2]);
		std::vector<uint8_t> crc_src;
		crc_src.reserve(8 + data_len - 2);
		crc_src.insert(crc_src.end(), head.begin(), head.end());
		crc_src.insert(crc_src.end(), payload.begin(), payload.end() - 2);
		const auto calc_crc = crc16_ibm(crc_src.data(), crc_src.size());
		if (recv_crc != calc_crc) {
			RCLCPP_ERROR(
				this->get_logger(),
				"CRC mismatch recv=0x%04X calc=0x%04X payload=%s",
				recv_crc,
				calc_crc,
				bytes_to_hex(payload).c_str());
			return false;
		}

		frame.flags = payload[0];
		frame.cmd = payload[1];
		frame.body.assign(payload.begin() + 2, payload.end() - 2);
		return true;
	}

	std::optional<ProtocolFrame> send_and_wait_resp(uint8_t cmd, const std::vector<uint8_t> &body)
	{
		if (!connect_if_needed()) {
			return std::nullopt;
		}

		const auto frame_bytes = build_frame(cmd, body);
		const auto sent = ::send(socket_fd_, frame_bytes.data(), frame_bytes.size(), 0);
		if (sent != static_cast<ssize_t>(frame_bytes.size())) {
			RCLCPP_ERROR(this->get_logger(), "Failed to send command cmd=0x%02X", cmd);
			close_socket();
			return std::nullopt;
		}

		ProtocolFrame resp;
		if (!recv_frame(resp)) {
			RCLCPP_ERROR(this->get_logger(), "No valid response for cmd=0x%02X", cmd);
			close_socket();
			return std::nullopt;
		}

		if ((resp.flags & kFlagIsResp) == 0U || (resp.flags & kFlagIsReport) != 0U) {
			RCLCPP_ERROR(this->get_logger(), "Invalid response flags: 0x%02X", resp.flags);
			return std::nullopt;
		}

		if (resp.cmd != cmd) {
			RCLCPP_WARN(this->get_logger(), "Response cmd mismatch req=0x%02X resp=0x%02X", cmd, resp.cmd);
		}

		return resp;
	}

	static std::vector<std::string> split_strings_with_nul(const std::vector<uint8_t> &data, size_t start)
	{
		std::vector<std::string> result;
		std::string current;
		for (size_t i = start; i < data.size(); ++i) {
			if (data[i] == '\0') {
				result.push_back(current);
				current.clear();
			} else {
				current.push_back(static_cast<char>(data[i]));
			}
		}
		if (!current.empty()) {
			result.push_back(current);
		}
		return result;
	}

	std::vector<uint8_t> build_start_app_body(const std::string &request)
	{
		int idx = -1;
		std::string app_id;
		std::string app_func;

		std::stringstream ss(request);
		std::string item;
		while (std::getline(ss, item, ',')) {
			const auto token = trim(item);
			const auto pos = token.find(':');
			if (pos == std::string::npos) {
				continue;
			}
			const auto key = trim(token.substr(0, pos));
			const auto value = trim(token.substr(pos + 1));

			if (key == "idx") {
				try {
					idx = std::stoi(value);
				} catch (...) {
					idx = -1;
				}
			} else if (key == "id") {
				app_id = value;
			} else if (key == "func") {
				app_func = value;
			}
		}

		std::vector<uint8_t> body;
		if (idx >= 0 && idx <= 254) {
			body.push_back(static_cast<uint8_t>(idx));
			if (!app_func.empty()) {
				body.insert(body.end(), app_func.begin(), app_func.end());
				body.push_back('\0');
			}
		} else {
			body.push_back(0xFF);
			if (!app_id.empty()) {
				body.insert(body.end(), app_id.begin(), app_id.end());
				body.push_back('\0');
			}
			if (!app_func.empty()) {
				body.insert(body.end(), app_func.begin(), app_func.end());
				body.push_back('\0');
			}
		}

		return body;
	}

	bool parse_key_value_request(const std::string &request, std::string &confirm, std::string &app_id)
	{
		confirm.clear();
		app_id.clear();

		std::stringstream ss(request);
		std::string item;
		while (std::getline(ss, item, ',')) {
			const auto token = trim(item);
			const auto pos = token.find(':');
			if (pos == std::string::npos) {
				continue;
			}
			const auto key = trim(token.substr(0, pos));
			const auto value = trim(token.substr(pos + 1));

			if (key == "confirm") {
				confirm = value;
			} else if (key == "id") {
				app_id = value;
			}
		}

		return !confirm.empty() && !app_id.empty();
	}

	void publish_status(const std::string &text)
	{
		std_msgs::msg::String msg;
		msg.data = text;
		status_pub_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
	}

	void publish_response_result(const std::string &action, const ProtocolFrame &resp)
	{
		const bool ok = (resp.flags & kFlagRespOk) != 0U;
		if (!ok) {
			std::string err = "unknown";
			if (!resp.body.empty()) {
				const auto code = static_cast<int>(resp.body[0]);
				std::string detail;
				if (resp.body.size() > 1) {
					detail.assign(resp.body.begin() + 1, resp.body.end());
				}
				err = "code=" + std::to_string(code) + ", msg=" + detail;
			}
			publish_status(action + " failed: " + err);
			return;
		}

		publish_status(action + " ok");
	}

	void on_start_app(const std_msgs::msg::String::SharedPtr msg)
	{
		if (!msg) {
			return;
		}
		const auto body = build_start_app_body(msg->data);
		const auto resp = send_and_wait_resp(CMD_START_APP, body);
		if (!resp.has_value()) {
			publish_status("start_app failed: no response");
			return;
		}
		publish_response_result("start_app", resp.value());
	}

	void on_exit_app(const std_msgs::msg::String::SharedPtr msg)
	{
		if (!msg) {
			publish_status("exit_app failed: empty message");
			return;
		}

		std::string confirm;
		std::string app_id;
		if (!parse_key_value_request(msg->data, confirm, app_id)) {
			publish_status("exit_app rejected: format must be 'confirm:EXIT_NOW,id:<app_id>'");
			return;
		}

		if (confirm != exit_confirm_word_) {
			publish_status("exit_app rejected: invalid confirm word");
			return;
		}

		std::vector<uint8_t> body;
		body.insert(body.end(), app_id.begin(), app_id.end());
		body.push_back('\0');

		const auto resp = send_and_wait_resp(CMD_EXIT_APP, body);
		if (!resp.has_value()) {
			publish_status("exit_app failed: no response");
			return;
		}
		publish_response_result("exit_app", resp.value());
	}

	void on_snap(const std_msgs::msg::Empty::SharedPtr)
	{
		const auto resp = send_and_wait_resp(CMD_CAMERA_SNAP, {});
		if (!resp.has_value()) {
			publish_status("snap failed: no response");
			return;
		}
		publish_response_result("snap", resp.value());
	}

	void on_query(const std_msgs::msg::String::SharedPtr msg)
	{
		if (!msg) {
			return;
		}

		const auto request = trim(msg->data);
		if (request == "app_list") {
			const auto resp = send_and_wait_resp(CMD_APP_LIST, {});
			if (!resp.has_value()) {
				publish_status("query app_list failed: no response");
				return;
			}
			if ((resp->flags & kFlagRespOk) == 0U) {
				publish_response_result("query app_list", resp.value());
				return;
			}

			std::ostringstream ss;
			const auto count = resp->body.empty() ? 0 : static_cast<int>(resp->body[0]);
			const auto app_ids = split_strings_with_nul(resp->body, 1);
			ss << "app_list count=" << count << " [";
			for (size_t i = 0; i < app_ids.size(); ++i) {
				if (i > 0) {
					ss << ",";
				}
				ss << app_ids[i];
			}
			ss << "]";
			publish_status(ss.str());
			return;
		}

		if (request == "cur_app_info") {
			const auto resp = send_and_wait_resp(CMD_CUR_APP_INFO, {});
			if (!resp.has_value()) {
				publish_status("query cur_app_info failed: no response");
				return;
			}
			if ((resp->flags & kFlagRespOk) == 0U) {
				publish_response_result("query cur_app_info", resp.value());
				return;
			}

			if (resp->body.empty()) {
				publish_status("cur_app_info empty");
				return;
			}

			const auto idx = static_cast<int>(resp->body[0]);
			const auto fields = split_strings_with_nul(resp->body, 1);
			std::ostringstream ss;
			ss << "cur_app_info idx=" << idx;
			if (!fields.empty()) {
				ss << " id=" << fields[0];
			}
			if (fields.size() > 1) {
				ss << " name=" << fields[1];
			}
			if (fields.size() > 2) {
				ss << " desc=" << fields[2];
			}
			publish_status(ss.str());
			return;
		}

		if (request.rfind("app_info:", 0) == 0) {
			const auto value = trim(request.substr(9));
			std::vector<uint8_t> body;
			if (value.rfind("idx=", 0) == 0) {
				int idx = -1;
				try {
					idx = std::stoi(value.substr(4));
				} catch (...) {
					idx = -1;
				}
				if (idx < 0 || idx > 254) {
					publish_status("query app_info invalid idx, use 0~254");
					return;
				}
				body.push_back(static_cast<uint8_t>(idx));
			} else if (value.rfind("id=", 0) == 0) {
				body.push_back(0xFF);
				const auto app_id = value.substr(3);
				body.insert(body.end(), app_id.begin(), app_id.end());
			} else {
				publish_status("query app_info format: app_info:idx=0 OR app_info:id=camera");
				return;
			}

			const auto resp = send_and_wait_resp(CMD_APP_INFO, body);
			if (!resp.has_value()) {
				publish_status("query app_info failed: no response");
				return;
			}
			if ((resp->flags & kFlagRespOk) == 0U) {
				publish_response_result("query app_info", resp.value());
				return;
			}

			if (resp->body.empty()) {
				publish_status("app_info empty");
				return;
			}

			const auto idx = static_cast<int>(resp->body[0]);
			const auto fields = split_strings_with_nul(resp->body, 1);
			std::ostringstream ss;
			ss << "app_info idx=" << idx;
			if (!fields.empty()) {
				ss << " id=" << fields[0];
			}
			if (fields.size() > 1) {
				ss << " name=" << fields[1];
			}
			if (fields.size() > 2) {
				ss << " desc=" << fields[2];
			}
			publish_status(ss.str());
			return;
		}

		publish_status("unsupported query, use app_list | cur_app_info | app_info:idx=0 | app_info:id=camera");
	}

private:
	std::string camera_host_;
	int camera_port_{5555};
	int recv_timeout_ms_{800};
	int socket_fd_{-1};

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exit_sub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr query_sub_;
	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr snap_sub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
	std::string exit_confirm_word_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraTcpNode>());
	rclcpp::shutdown();
	return 0;
}
