#ifndef MOTION_MANAGER__MOTION_SERIAL_HPP
#define MOTION_MANAGER__MOTION_SERIAL_HPP
#include <string>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "serial/serial.h"

#include "system_state/state.hpp"



namespace controller
{

    typedef struct
    {
        std::string port = "/dev/ttyACM0"; // 串口配置信息
        uint32_t baudrate = 115200;        // 波特率
        uint32_t timeout_ms = 10;           // 读取超时时间
        uint16_t buffer_size = 100;        // 串口读取缓冲区大小

    } serial_info;

    /**
     * @brief 运动控制的通用串口，需要实现对底层串口的打开、关闭、读取、写入等操作
     */
    class MotionSerial : public rclcpp::Node
    {
    
    protected:
        serial_info si_;                 // 串口信息
        serial::Serial serial_;          // 串口对象
        bool serial_enable = false;      // 串口是否可用
        uint8_t *buffer;                 // 串口读取缓冲区
        bool read_serial_enable = false; // 是否开启循环读取串口数据

        

    public:
        explicit MotionSerial();
        ~MotionSerial();
        uint64_t OpenSerial();
        /**
         * @brief 手动关闭串口
         */
        void CloseSerial() { this->serial_.close(); }
        /**
         * @brief 启用循环读取串口数据
         */
        void EnableReadSerial() { this->read_serial_enable = true; }
        /**
         * @brief 禁用循环读取串口数据
         */
        void DisableReadSerial() { this->read_serial_enable = false; }
    };

}
#endif // MOTION_MANAGER__MOTION_SERIAL_HPP