#ifndef MOTION_MANAGER__MOTION_MANAGER_HPP
#define MOTION_MANAGER__MOTION_MANAGER_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "system_state/state.hpp"
#include "system_state/error.hpp"
#include "state_interfaces/msg/speed.hpp"

#include "serial/serial.h"

namespace controller
{

    typedef struct
    {
        std::string port = "/dev/ttyUSB0"; //
        uint32_t baudrate = 115200;
        uint32_t timeout_ms = 1000;
    } serial_info;

    /**
     * 运动控制管理模块
     */
    class MotionManager : public rclcpp::Node
    {
    private:
        serial_info serial_info_;   // 串口配置信息
        serial::Serial serial_;     // motion serial ptr
        bool serial_enable = false; // 串口是否可用

        uint8_t serial_read_buffer[100];                          // 串口读取缓冲区
        const TIME_MS serial_reader_timer_interval = TIME_MS(10); // 串口读取定时器回调间隔 单位：毫秒
        rclcpp::TimerBase::SharedPtr serial_reader_timer_;        // 串口读取定时器
        void serial_reader_timer_callback();

        


    public:
        explicit MotionManager();
        ~MotionManager();
        uint64_t TryOpenSerial();
        bool SetSpeed(state_interfaces::msg::Speed speed);
    };
} // namespace controll
#endif // MOTION_MANAGER__MOTION_MANAGER_HPP