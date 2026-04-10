#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <custom_interfaces/msg/robot_state.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

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

constexpr uint8_t CMD_CAMERA_SNAP = 0x01;
constexpr uint8_t APP_CMD_VISION_START = 0x10;
constexpr uint8_t APP_CMD_VISION_STOP = 0x11;
constexpr uint8_t APP_CMD_VISION_STATUS = 0x12;

constexpr uint32_t kHeader = 0xBBACCAAA;
constexpr size_t kTotalServoCount = 6;
constexpr size_t kDurationIndex = 6;
constexpr char kVisionStateName[] = "VISION_TASK";
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
        status_poll_ms_ = this->declare_parameter<int>("status_poll_ms", 100);
        exit_confirm_word_ = this->declare_parameter<std::string>("exit_confirm_word", "EXIT_NOW");
        forward_only_in_vision_task_ = this->declare_parameter<bool>("forward_only_in_vision_task", true);
        vision_state_name_ = this->declare_parameter<std::string>("vision_state_name", kVisionStateName);
        log_forwarded_frames_ = this->declare_parameter<bool>("log_forwarded_frames", false);

        const auto default_pwms = this->declare_parameter<std::vector<int64_t>>(
            "default_arm_pwms",
            std::vector<int64_t>{1500, 1350, 2300, 1500, 1500, 1500});
        load_default_pwms(default_pwms);

        const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/status/camera/protocol", status_qos);
        connection_pub_ = this->create_publisher<std_msgs::msg::String>("/status/camera/connection", status_qos);
        vision_app_state_pub_ = this->create_publisher<std_msgs::msg::String>("/status/camera/vision_app_state", status_qos);
        vision_track_state_pub_ = this->create_publisher<std_msgs::msg::String>("/status/camera/vision_track_state", status_qos);
        target_found_pub_ = this->create_publisher<std_msgs::msg::Bool>("/status/camera/target_found", status_qos);
        can_scan_pub_ = this->create_publisher<std_msgs::msg::Bool>("/status/camera/can_scan", status_qos);
        target_pixel_pub_ = this->create_publisher<std_msgs::msg::String>("/status/camera/target_pixel", status_qos);
        direct_pwm_pub_ = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
            "/cmd/arm/direct_pwm_command",
            10);

        start_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/camera/start_app",
            10,
            std::bind(&CameraTcpNode::on_start_app, this, std::placeholders::_1));

        exit_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/camera/exit_app",
            10,
            std::bind(&CameraTcpNode::on_exit_app, this, std::placeholders::_1));

        vision_start_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/camera/vision/start",
            10,
            std::bind(&CameraTcpNode::on_vision_start, this, std::placeholders::_1));

        vision_stop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/camera/vision/stop",
            10,
            std::bind(&CameraTcpNode::on_vision_stop, this, std::placeholders::_1));

        query_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cmd/camera/query",
            10,
            std::bind(&CameraTcpNode::on_query, this, std::placeholders::_1));

        snap_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/cmd/camera/snap",
            10,
            std::bind(&CameraTcpNode::on_snap, this, std::placeholders::_1));

        robot_state_sub_ = this->create_subscription<custom_interfaces::msg::RobotState>(
            "/robot/state",
            10,
            std::bind(&CameraTcpNode::on_robot_state, this, std::placeholders::_1));

        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(std::max(50, status_poll_ms_)),
            std::bind(&CameraTcpNode::poll_vision_status, this));

        publish_connection_state(false);
        publish_vision_app_state("DISCONNECTED");
        publish_vision_track_state("DISCONNECTED");
    }

    ~CameraTcpNode() override
    {
        close_socket();
    }

private:
    struct VisionServoFrame
    {
        std::array<int, kTotalServoCount> pwm_values{};
        std::array<bool, kTotalServoCount> touched{};
        int duration_ms{0};
    };

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

    static void append_u16_le(std::vector<uint8_t> &out, uint16_t value)
    {
        out.push_back(static_cast<uint8_t>(value & 0xFF));
        out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
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

    static bool parse_fixed_digits(const std::string &text, size_t start, size_t width, int &value)
    {
        if (start + width > text.size()) {
            return false;
        }

        int parsed = 0;
        for (size_t i = 0; i < width; ++i) {
            const char ch = text[start + i];
            if (!std::isdigit(static_cast<unsigned char>(ch))) {
                return false;
            }
            parsed = parsed * 10 + (ch - '0');
        }
        value = parsed;
        return true;
    }

    static bool parse_digits_until(
        const std::string &text,
        size_t start,
        char delimiter,
        int &value,
        size_t &delimiter_pos)
    {
        if (start >= text.size()) {
            return false;
        }

        int parsed = 0;
        bool has_digit = false;
        size_t cursor = start;
        while (cursor < text.size() && std::isdigit(static_cast<unsigned char>(text[cursor]))) {
            has_digit = true;
            parsed = parsed * 10 + (text[cursor] - '0');
            ++cursor;
        }

        if (!has_digit || cursor >= text.size() || text[cursor] != delimiter) {
            return false;
        }

        value = parsed;
        delimiter_pos = cursor;
        return true;
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

    static bool starts_with(const std::string &value, const std::string &prefix)
    {
        return value.rfind(prefix, 0) == 0;
    }

    void load_default_pwms(const std::vector<int64_t> &defaults)
    {
        last_arm_pwms_ = {1500, 1350, 2300, 1500, 1500, 1500};
        for (size_t i = 0; i < std::min(defaults.size(), last_arm_pwms_.size()); ++i) {
            last_arm_pwms_[i] = static_cast<int>(std::clamp<int64_t>(defaults[i], 500, 2500));
        }
    }

    void on_robot_state(const custom_interfaces::msg::RobotState::SharedPtr msg)
    {
        if (!msg) {
            return;
        }
        vision_task_active_ = (msg->state_name == vision_state_name_);
    }

    bool connect_if_needed()
    {
        if (socket_fd_ >= 0) {
            return true;
        }

        socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            publish_connection_state(false);
            return false;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(static_cast<uint16_t>(camera_port_));
        if (::inet_pton(AF_INET, camera_host_.c_str(), &server_addr.sin_addr) <= 0) {
            publish_status("invalid camera_host: " + camera_host_);
            close_socket();
            publish_connection_state(false);
            return false;
        }

        if (::connect(socket_fd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
            close_socket();
            publish_connection_state(false);
            return false;
        }

        timeval timeout{};
        timeout.tv_sec = recv_timeout_ms_ / 1000;
        timeout.tv_usec = (recv_timeout_ms_ % 1000) * 1000;
        if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
            publish_status("failed to set recv timeout, continue with default");
        }

        publish_connection_state(true);
        publish_status("camera tcp connected: " + camera_host_ + ":" + std::to_string(camera_port_));
        return true;
    }

    void close_socket()
    {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
        }
        publish_connection_state(false);
    }

    bool recv_exact(std::vector<uint8_t> &out, size_t size)
    {
        out.clear();
        out.resize(size, 0);
        size_t total = 0;
        while (total < size) {
            const auto n = ::recv(socket_fd_, out.data() + total, size - total, 0);
            if (n <= 0) {
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
        data.push_back(kProtocolVersion);
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
            publish_status("invalid frame header");
            return false;
        }

        const uint32_t data_len = read_u32_le(head.data() + 4);
        if (data_len < 4) {
            publish_status("invalid data_len");
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
            publish_status("crc mismatch payload=" + bytes_to_hex(payload));
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
            publish_status("failed to send command cmd=0x" + to_hex(cmd));
            close_socket();
            return std::nullopt;
        }

        while (true) {
            ProtocolFrame resp;
            if (!recv_frame(resp)) {
                close_socket();
                publish_vision_app_state("DISCONNECTED");
                publish_vision_track_state("DISCONNECTED");
                return std::nullopt;
            }

            if ((resp.flags & kFlagIsReport) != 0U) {
                continue;
            }

            if ((resp.flags & kFlagIsResp) == 0U) {
                continue;
            }

            return resp;
        }
    }

    std::string to_hex(uint8_t value) const
    {
        std::ostringstream ss;
        ss << std::hex << static_cast<int>(value);
        return ss.str();
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

    std::vector<uint8_t> build_vision_start_body(const std::string &request)
    {
        int yaw_pwm = -1;
        int pitch_pwm = -1;

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
            try {
                if (key == "yaw_pwm") {
                    yaw_pwm = std::stoi(value);
                } else if (key == "pitch_pwm") {
                    pitch_pwm = std::stoi(value);
                }
            } catch (...) {
                continue;
            }
        }

        if (yaw_pwm < 500 || yaw_pwm > 2500 || pitch_pwm < 500 || pitch_pwm > 2500) {
            return {};
        }

        std::vector<uint8_t> body;
        body.reserve(5);
        body.push_back(0xFE);
        append_u16_le(body, static_cast<uint16_t>(yaw_pwm));
        append_u16_le(body, static_cast<uint16_t>(pitch_pwm));
        return body;
    }

    void publish_status(const std::string &text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        status_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
    }

    void publish_connection_state(bool connected)
    {
        if (connected == connection_state_) {
            return;
        }
        connection_state_ = connected;
        std_msgs::msg::String msg;
        msg.data = connected ? "CONNECTED" : "DISCONNECTED";
        connection_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "camera connection state: %s", msg.data.c_str());
    }

    void publish_vision_app_state(const std::string &state)
    {
        if (state == last_app_state_) {
            return;
        }
        last_app_state_ = state;
        std_msgs::msg::String msg;
        msg.data = state;
        vision_app_state_pub_->publish(msg);
    }

    void publish_vision_track_state(const std::string &state)
    {
        if (state == last_track_state_) {
            return;
        }
        last_track_state_ = state;
        std_msgs::msg::String msg;
        msg.data = state;
        vision_track_state_pub_->publish(msg);
    }

    void publish_target_found(bool found)
    {
        if (found == last_target_found_) {
            return;
        }
        last_target_found_ = found;
        std_msgs::msg::Bool msg;
        msg.data = found;
        target_found_pub_->publish(msg);
    }

    void publish_can_scan(bool can_scan)
    {
        if (can_scan == last_can_scan_) {
            return;
        }
        last_can_scan_ = can_scan;
        std_msgs::msg::Bool msg;
        msg.data = can_scan;
        can_scan_pub_->publish(msg);
    }

    void publish_target_pixel(const std::string &pixel)
    {
        if (pixel == last_target_pixel_) {
            return;
        }
        last_target_pixel_ = pixel;
        std_msgs::msg::String msg;
        msg.data = pixel;
        target_pixel_pub_->publish(msg);
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

    void on_vision_start(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<uint8_t> body;
        if (msg) {
            body = build_vision_start_body(msg->data);
        }

        const auto resp = send_and_wait_resp(APP_CMD_VISION_START, body);
        if (!resp.has_value()) {
            publish_status("vision_start failed: no response");
            return;
        }
        publish_response_result("vision_start", resp.value());
    }

    void on_vision_stop(const std_msgs::msg::String::SharedPtr)
    {
        const auto resp = send_and_wait_resp(APP_CMD_VISION_STOP, {});
        if (!resp.has_value()) {
            publish_status("vision_stop failed: no response");
            return;
        }
        publish_response_result("vision_stop", resp.value());
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
        if (request == "vision_status") {
            request_vision_status();
            return;
        }

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

        if (starts_with(request, "app_info:")) {
            const auto value = trim(request.substr(9));
            std::vector<uint8_t> body;
            if (starts_with(value, "idx=")) {
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
            } else if (starts_with(value, "id=")) {
                body.push_back(0xFF);
                const auto app_id = value.substr(3);
                body.insert(body.end(), app_id.begin(), app_id.end());
            } else {
                publish_status("query app_info format: app_info:idx=0 OR app_info:id=mygo_pipeline_uart");
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

        publish_status("unsupported query, use vision_status | app_list | cur_app_info | app_info:idx=0 | app_info:id=mygo_pipeline_uart");
    }

    void poll_vision_status()
    {
        request_vision_status();
    }

    void request_vision_status()
    {
        const auto resp = send_and_wait_resp(APP_CMD_VISION_STATUS, {});
        if (!resp.has_value()) {
            publish_vision_app_state("DISCONNECTED");
            publish_vision_track_state("DISCONNECTED");
            publish_target_found(false);
            publish_can_scan(false);
            publish_target_pixel("-1,-1");
            return;
        }

        if ((resp->flags & kFlagRespOk) == 0U) {
            publish_response_result("vision_status", resp.value());
            return;
        }

        const auto fields = split_strings_with_nul(resp->body, 0);
        if (fields.size() < 6) {
            publish_status("vision_status invalid body");
            return;
        }

        const std::string &app_state = fields[0];
        const std::string &track_state = fields[1];
        const bool active = (fields[2] == "1");
        const bool target_found = (fields[3] == "1");
        const std::string &command = fields[5];

        publish_vision_app_state(app_state);
        publish_vision_track_state(track_state);
        publish_target_found(target_found);

        if (fields.size() > 7) {
            publish_can_scan(fields[7] == "1");
        }
        if (fields.size() > 9) {
            publish_target_pixel(fields[8] + "," + fields[9]);
        }

        if (!active || command.empty()) {
            return;
        }

        if (forward_only_in_vision_task_ && !vision_task_active_) {
            return;
        }

        auto parsed = parse_vision_frame(command);
        if (!parsed.has_value()) {
            publish_status("invalid vision command frame: " + command);
            return;
        }
        publish_direct_pwm(parsed.value(), command);
    }

    std::optional<VisionServoFrame> parse_vision_frame(const std::string &frame) const
    {
        if (frame.size() < 2 || frame.front() != '{' || frame.back() != '}') {
            return std::nullopt;
        }

        // 新版透传格式: {P1209T0050P1350T1000...}
        // 顺序映射为舵机 0..N-1；多余项忽略，缺失项保持 last_arm_pwms_。
        if (frame.find('#') == std::string::npos) {
            VisionServoFrame parsed{};
            size_t cursor = 1;
            size_t servo_id = 0;
            int min_duration_ms = std::numeric_limits<int>::max();

            while (cursor + 1 < frame.size() && servo_id < kTotalServoCount) {
                if (frame[cursor] != 'P') {
                    ++cursor;
                    continue;
                }

                int pwm = 0;
                int duration_ms = 0;
                size_t duration_mark = 0;
                if (!parse_digits_until(frame, cursor + 1, 'T', pwm, duration_mark)) {
                    return std::nullopt;
                }

                size_t end_mark = duration_mark + 1;
                if (end_mark >= frame.size() || !std::isdigit(static_cast<unsigned char>(frame[end_mark]))) {
                    return std::nullopt;
                }

                duration_ms = 0;
                while (end_mark < frame.size() && std::isdigit(static_cast<unsigned char>(frame[end_mark]))) {
                    duration_ms = duration_ms * 10 + (frame[end_mark] - '0');
                    ++end_mark;
                }

                parsed.pwm_values[servo_id] = std::clamp(pwm, 500, 2500);
                parsed.touched[servo_id] = true;
                if (duration_ms > 0) {
                    min_duration_ms = std::min(min_duration_ms, duration_ms);
                }

                ++servo_id;
                cursor = end_mark;
            }

            if (servo_id == 0) {
                return std::nullopt;
            }

            parsed.duration_ms = (min_duration_ms == std::numeric_limits<int>::max()) ? 50 : min_duration_ms;
            return parsed;
        }

        VisionServoFrame parsed{};
        size_t cursor = 1;
        bool has_any_servo = false;

        while (cursor + 1 < frame.size()) {
            if (frame[cursor] != '#') {
                ++cursor;
                continue;
            }

            int servo_id = 0;
            int pwm = 0;
            int duration_ms = 0;
            if (!parse_fixed_digits(frame, cursor + 1, 3, servo_id)) {
                return std::nullopt;
            }

            const size_t pwm_mark = cursor + 4;
            if (pwm_mark >= frame.size() || frame[pwm_mark] != 'P') {
                return std::nullopt;
            }
            size_t duration_mark = 0;
            if (!parse_digits_until(frame, pwm_mark + 1, 'T', pwm, duration_mark)) {
                return std::nullopt;
            }

            size_t end_mark = 0;
            if (!parse_digits_until(frame, duration_mark + 1, '!', duration_ms, end_mark)) {
                return std::nullopt;
            }

            if (servo_id >= 0 && servo_id < static_cast<int>(kTotalServoCount)) {
                parsed.pwm_values[static_cast<size_t>(servo_id)] = std::clamp(pwm, 500, 2500);
                parsed.touched[static_cast<size_t>(servo_id)] = true;
                parsed.duration_ms = std::max(parsed.duration_ms, duration_ms);
                has_any_servo = true;
            }
            cursor = end_mark + 1;
        }

        if (!has_any_servo) {
            return std::nullopt;
        }
        return parsed;
    }

    void publish_direct_pwm(const VisionServoFrame &frame, const std::string &raw_frame)
    {
        for (size_t servo_id = 0; servo_id < kTotalServoCount; ++servo_id) {
            if (frame.touched[servo_id]) {
                last_arm_pwms_[servo_id] = frame.pwm_values[servo_id];
            }
        }

        example_interfaces::msg::Float64MultiArray msg;
        msg.data.reserve(kTotalServoCount + 1);
        for (const int pwm : last_arm_pwms_) {
            msg.data.push_back(static_cast<double>(pwm));
        }
        msg.data.push_back(static_cast<double>(std::max(frame.duration_ms, 20)));
        direct_pwm_pub_->publish(msg);

        if (log_forwarded_frames_) {
            RCLCPP_INFO(this->get_logger(), "forwarded vision tcp frame: %s", raw_frame.c_str());
        }
    }

    std::string camera_host_;
    int camera_port_{5555};
    int recv_timeout_ms_{3000};
    int status_poll_ms_{100};
    int socket_fd_{-1};
    bool connection_state_{true};
    bool forward_only_in_vision_task_{true};
    bool vision_task_active_{false};
    bool log_forwarded_frames_{false};
    std::string vision_state_name_{kVisionStateName};
    std::string last_app_state_{"UNKNOWN"};
    std::string last_track_state_{"UNKNOWN"};
    bool last_target_found_{false};
    bool last_can_scan_{false};
    std::string last_target_pixel_{"-1,-1"};
    std::string exit_confirm_word_;

    std::array<int, kTotalServoCount> last_arm_pwms_{};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exit_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_start_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr query_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr snap_sub_;
    rclcpp::Subscription<custom_interfaces::msg::RobotState>::SharedPtr robot_state_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr connection_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_app_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vision_track_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_found_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_scan_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_pixel_pub_;
    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr direct_pwm_pub_;

    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTcpNode>());
    rclcpp::shutdown();
    return 0;
}
