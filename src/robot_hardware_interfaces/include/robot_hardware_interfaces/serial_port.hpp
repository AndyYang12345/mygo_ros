#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <cstdint>
#include <string>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& device, int baudrate);
    void close();
    bool is_open() const;

    bool set_baudrate(int baudrate);

    bool write_bytes(const uint8_t* data, size_t size);
    bool write_string(const std::string& data);

private:
    int fd_;
    std::string device_;

    bool configure_port(int baudrate);
    static int to_termios_baud(int baudrate);
};

#endif // SERIAL_PORT_HPP
