#include "motion_controller/motion_serial.hpp"

namespace controller {

MotionSerial::MotionSerial() : Node("motion_serial") { this->buffer = new uint8_t[this->si_.buffer_size]; }

MotionSerial::~MotionSerial() { delete[] this->buffer; }

uint64_t MotionSerial::OpenSerial() {
    bool port_existed = false;
    {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        for (size_t i = 0; i < devices_found.size(); i++) {
            if (devices_found[i].port == this->si_.port) {
                port_existed = true;
                break;
            }
        }
    }
    if (!port_existed) {
        return static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_PORT_NOT_FOUND);
    } else {
        try {
            this->serial_.setPort(this->si_.port);
            this->serial_.setBaudrate(this->si_.baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(this->si_.timeout_ms);
            this->serial_.setTimeout(to);
            this->serial_.open();
            if (this->serial_.isOpen()) {
                this->serial_enable = true;
                return static_cast<uint64_t>(SystemState::ErrorCode::NO_ERROR);
            } else {
                return static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_OPEN_FAILED);
            }
        } catch (const std::exception& e) {

            return static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_UNKOOWN_ERROR);
        }
    }
}


} // namespace controller