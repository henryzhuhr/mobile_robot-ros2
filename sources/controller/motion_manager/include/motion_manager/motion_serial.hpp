#ifndef MOTION_MANAGER__MOTION_SERIAL_HPP
#define MOTION_MANAGER__MOTION_SERIAL_HPP

#include <memory>
#include <string>

#include "serial/serial.h"

typedef struct
{
    std::string port = "/dev/ttyUSB0";
    uint32_t baudrate = 115200;
    uint32_t timeout_ms = 1000;
} serial_info;

/**
 * 运动控制串口
 */
class MotionSerial
{
private:
    std::shared_ptr<serial::Serial> m_serial_; // motion serial ptr
public:
    MotionSerial(/* args */);
    ~MotionSerial();
};

#endif // MOTION_MANAGER__MOTION_SERIAL_HPP