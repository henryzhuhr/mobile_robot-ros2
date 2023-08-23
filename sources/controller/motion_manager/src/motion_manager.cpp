#include "motion_manager/motion_manager.hpp"

#include <string>
#include <vector>
namespace controller
{
    MotionManager::MotionManager(/* args */) : Node("motion_serial")
    {

        // 初始化定时器
        this->serial_reader_timer_ = this->create_wall_timer(
            this->serial_reader_timer_interval, std::bind(&MotionManager::serial_reader_timer_callback, this));
    }

    MotionManager::~MotionManager()
    {
    }

    uint64_t MotionManager::TryOpenSerial()
    {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
// #define DEBUG
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Found %ld devices", devices_found.size());
        for (auto &device : devices_found)
        {
            RCLCPP_INFO(this->get_logger(), "      Device: %s", device.port.c_str());
        }
#endif

        auto iter = std::find_if(devices_found.begin(), devices_found.end(), [&](serial::PortInfo &device)
                                 { return device.port == this->serial_info_.port; });
        if (iter == devices_found.end())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "\033[01;31mNot found serial (error: %ld):\033[0m %s",
                         static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_PORT_NOT_FOUND),
                         this->serial_info_.port.c_str());
            return static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_PORT_NOT_FOUND);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "\033[01;32mFound serial: %s\033[0m",
                        this->serial_info_.port.c_str());
            try
            {
                this->serial_.setPort(this->serial_info_.port);
                this->serial_.setBaudrate(this->serial_info_.baudrate);
                serial::Timeout to = serial::Timeout::simpleTimeout(this->serial_info_.timeout_ms);
                this->serial_.setTimeout(to);
                this->serial_.open();
                if (this->serial_.isOpen())
                {
                    this->serial_enable = true;

                    RCLCPP_INFO(this->get_logger(), "\033[01;32mSuccessfully open serial:\033[0m %s",
                                this->serial_info_.port.c_str());
                    RCLCPP_INFO(this->get_logger(), "\033[01;32mEnable serial:\033[0m %s (%d)",
                                this->serial_info_.port.c_str(), this->serial_enable);
                    return static_cast<uint64_t>(SystemState::ErrorCode::NO_ERROR);
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "\033[01;31mOpen serial failed (error: %ld):\033[0m %s",
                                 static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_OPEN_FAILED),
                                 this->serial_info_.port.c_str());
                    return static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_OPEN_FAILED);
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "\033[01;31mOpen serial failed(error: %ld):\033[0m %s",
                             static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_UNKOOWN_ERROR),
                             this->serial_info_.port.c_str());
                return static_cast<uint64_t>(SystemState::ErrorCode::SERIAL_UNKOOWN_ERROR);
            }
        }
    }

    void MotionManager::serial_reader_timer_callback()
    {

        if (this->serial_enable)
        {
            size_t n = this->serial_.available();
            this->serial_.read(this->serial_read_buffer, n);
            for (size_t i = 0; i < n; i++)
            {
                // 16进制的方式打印到屏幕
                std::cout << std::hex << (this->serial_read_buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;

            uint8_t serial_write_buffer[100]; // 串口读取缓冲区
            uint8_t _cnt = 18;
            for (uint8_t i = 0; i < _cnt; i++)
            {
                serial_write_buffer[i] = 0x00;
            }
            serial_write_buffer[0] = 0xAA;
            serial_write_buffer[1] = 0x55;
            serial_write_buffer[4] = 0x0A;
            serial_write_buffer[16] = 0x0A;
            serial_write_buffer[17] = 0xFF;

            this->serial_.write(serial_write_buffer, _cnt);
            // std::string seiral_buffer(serial_write_buffer, _cnt);
            std::string seiral_buffer_str = "";
            for (uint8_t i = 0; i < _cnt; i++)
            {
                seiral_buffer_str += std::to_string(serial_write_buffer[i]);
                seiral_buffer_str += " ";
            }
            RCLCPP_INFO(this->get_logger(), "serial write: %s",seiral_buffer_str.c_str());
        }
    }
} // namespace controll