#include "robot_hardware_interfaces/send_command.hpp"

#include <algorithm>
#include <array>
#include <cctype>

namespace {
struct ModeDefault {
    const char *mode;
    const char *device;
    int baudrate;
};

constexpr std::array<ModeDefault, 5> kModeDefaults = {{
    {"CHASSIS", "/dev/ttyUSB1", 115200},
    {"ARM", "/dev/ttyACM2", 115200},
    {"CAMERA", "/dev/ttyUSB2", 115200},
    {"GRIPPER", "/dev/ttyUSB3", 115200},
    {"POLE", "/dev/ttyUSB4", 115200},
}};
}

SendCommand::SendCommand(const std::string &mode_name)
{
    config_.mode_name = normalize_mode_name(mode_name);
    if (!resolve_config_for_mode(config_.mode_name, config_))
    {
        config_.mode_name = "CHASSIS";
        resolve_config_for_mode(config_.mode_name, config_);
    }
}

bool SendCommand::initialize()
{
    return serial_port_.open(config_.device, config_.baudrate);
}

bool SendCommand::send(const std::string &payload)
{
    if (!serial_port_.is_open())
    {
        return false;
    }
    return serial_port_.write_string(payload);
}

bool SendCommand::is_ready() const
{
    return serial_port_.is_open();
}

const SendCommand::SerialConfig &SendCommand::config() const
{
    return config_;
}

std::string SendCommand::normalize_mode_name(const std::string &mode_name)
{
    std::string normalized = mode_name;
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return normalized;
}

bool SendCommand::resolve_config_for_mode(const std::string &mode_name, SerialConfig &out_config)
{
    const auto normalized = normalize_mode_name(mode_name);
    for (const auto &item : kModeDefaults)
    {
        if (normalized == item.mode)
        {
            out_config.mode_name = item.mode;
            out_config.device = item.device;
            out_config.baudrate = item.baudrate;
            return true;
        }
    }
    return false;
}
