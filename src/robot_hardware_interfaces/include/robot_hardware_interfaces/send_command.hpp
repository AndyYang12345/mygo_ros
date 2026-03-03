#ifndef SEND_COMMAND_HPP
#define SEND_COMMAND_HPP

#include <string>

#include "robot_hardware_interfaces/serial_port.hpp"

class SendCommand {
public:
    struct SerialConfig {
        std::string mode_name;
        std::string device;
        int baudrate;
    };

    explicit SendCommand(const std::string &mode_name);

    bool initialize();
    bool send(const std::string &payload);
    bool is_ready() const;

    const SerialConfig &config() const;

private:
    static std::string normalize_mode_name(const std::string &mode_name);
    static bool resolve_config_for_mode(const std::string &mode_name, SerialConfig &out_config);

    SerialConfig config_;
    SerialPort serial_port_;
};

#endif  // SEND_COMMAND_HPP
