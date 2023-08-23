#include "motion_manager/motion_serial.hpp"

#include <vector>

MotionSerial::MotionSerial(/* args */)
{

    serial_info si;
    // std::vector<serial::PortInfo> devices_found = serial::list_ports();
    m_serial_ = std::make_shared<serial::Serial>(
        si.port, si.baudrate, serial::Timeout::simpleTimeout(si.timeout_ms));
}

MotionSerial::~MotionSerial()
{
}
