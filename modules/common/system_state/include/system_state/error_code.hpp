#ifndef COMMON__SYSTEM_STATE__ERROR_CODE_HPP
#define COMMON__SYSTEM_STATE__ERROR_CODE_HPP

#include <cstdint>
namespace SystemState {
/**
 * @brief 错误码 (uint64_t)
 */
enum class ErrorCode : uint64_t {
    NO_ERROR = 0,                // 没有错误
    STATE_UPDATE_GROUP_OVERFLOW, // 系统状态更新时，分组超出范围
    STATE_UPDATE_GROUP_NOT_FOUND, // 系统状态更新时，未找到对应的状态分组
    STATE_UPDATE_ID_OVERFLOW, // 系统状态更新时，状态ID超出范围
    STATE_UPDATE_ID_NOT_FOUND, // 系统状态更新时，未找到对应的状态

    // 文件相关错误码
    FILE_NOT_FOUND, // 文件未找到

    // 解析配置错误码
    CONFIG__KEY_NOT_FOUND, // 配置文件中未找到对应的key

    // ROS相关错误
    ROS_RCLCPP_NOT_INIT, // ROS rclcpp未初始化

    // 串口相关错误码
    SERIAL_PORT_NOT_FOUND,      // 串口未找到
    SERIAL_OPEN_FAILED,         // 串口打开失败
    SERIAL_WRITE_FAILED,        // 串口写入失败
    SERIAL_READ_FAILED,         // 串口读取失败
    SERIAL_TIMEOUT,             // 串口超时
    SERIAL_DATA_ERROR,          // 串口数据错误
    SERIAL_DATA_LENGTH_ERROR,   // 串口数据长度错误
    SERIAL_DATA_CRC_ERROR,      // 串口数据CRC校验错误
    SERIAL_DATA_ID_ERROR,       // 串口数据ID错误
    SERIAL_DATA_TYPE_ERROR,     // 串口数据类型错误
    SERIAL_DATA_VALUE_ERROR,    // 串口数据值错误
    SERIAL_DATA_NOT_FOUND,      // 串口数据未找到
    SERIAL_DATA_NOT_INIT,       // 串口数据未初始化
    SERIAL_DATA_NOT_SUPPORT,    // 串口数据不支持
    SERIAL_DATA_NOT_MATCH,      // 串口数据不匹配
    SERIAL_DATA_NOT_ENOUGH,     // 串口数据不足
    SERIAL_DATA_OVERFLOW,       // 串口数据溢出
    SERIAL_DATA_NOT_READY,      // 串口数据未准备好
    SERIAL_DATA_NOT_OPEN,       // 串口数据未打开
    SERIAL_DATA_NOT_CONNECTED,  // 串口数据未连接
    SERIAL_DATA_NOT_CONFIGURED, // 串口数据未配置
    SERIAL_DATA_NOT_STARTED,    // 串口数据未启动
    SERIAL_DATA_NOT_STOPPED,    // 串口数据未停止
    SERIAL_UNKOOWN_ERROR,       // 串口未知错误
};
} // namespace SystemState

#endif // COMMON__SYSTEM_STATE__ERROR_CODE_HPP