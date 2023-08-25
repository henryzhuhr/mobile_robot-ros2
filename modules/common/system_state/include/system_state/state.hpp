#ifndef COMMON__SYSTEM_STATE__STATE_HPP
#define COMMON__SYSTEM_STATE__STATE_HPP

#include "system_state/types.hpp"

namespace SystemState
{
    static const uint8_t GROUP_NUM_MAX = sizeof(uint8_t) << 3;
    static const uint8_t ID_NUM_MAX = sizeof(uint64_t) << 3;
    /**
     * @brief 系统状态分组，用于区分系统状态，便于修改
     */
    enum class StateGroup : uint8_t
    {
        TASK = 0,
        SENSOR,
        VISION,
    };

    /**
     * 系统任务列表 至多 64 个
     * 枚举序号标识 位 0~63
     * 任务状态以独占形式进行管理，即当前如果有任务在执行时，新的任务指令将返回reject状态
     */
    enum class Task : uint8_t
    {
        IDLE = 0,        // 空闲状态
        JOY_CONTROL = 1, // 手柄控制任务
    };
    /**
     * 系统预设传感器列表 至多 64 个
     * 枚举序号标识 位 0~63
     */
    enum class Sensor : uint8_t
    {
        NONE = 0,
        JOY = 1,        // 手柄
        USB_RGB_CAMERA, // USB RGB相机
        SCI_RGB_CAMERA, // SCI RGB相机
    };
    /**
     * 系统预设视觉算法列表 至多 64 个
     * 枚举序号标识 位 0~63
     */
    enum class Vison : uint8_t
    {
        NONE = 0,
        OBJECT_DETECTION = 1, // 目标检测
        LANE_DETECTION = 4,   // 车道线检测

    };

    namespace Color
    {
        static constexpr char DEFAULT[] = "\033[0m";
        static constexpr char GREEN[] = "\033[00;32m";
        static constexpr char LGREEN[] = "\033[01;32m";
        static constexpr char CYAN[] = "\033[00;36m";
        static constexpr char LCYAN[] = "\033[01;36m";
        static constexpr char RED[] = "\033[00;31m";
        static constexpr char LRED[] = "\033[01;31m";
        static constexpr char BLUE[] = "\033[00;34m";
        static constexpr char LBLUE[] = "\033[01;34m";

    }

}
namespace SystemState
{
    /**
     * @brief 错误码 (uint64_t)
     */
    enum class ErrorCode : uint64_t
    {
        NO_ERROR = 0,                 // 没有错误
        STATE_UPDATE_GROUP_NOT_FOUND, // 系统状态更新时，未找到对应的状态分组
        STATE_UPDATE_ID_OVERFLOW,     // 系统状态更新时，状态ID超出范围

        // 文件相关错误码
        FILE_NOT_FOUND, // 文件未找到

        // 解析配置错误码
        CONFIG__KEY_NOT_FOUND, // 配置文件中未找到对应的key

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
}
#endif // COMMON__SYSTEM_STATE__STATE_HPP