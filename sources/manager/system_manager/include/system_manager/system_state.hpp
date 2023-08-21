#ifndef SYSTEM_MANAGER__SYSTEM_MANAGER__SYSTEM_STATE_HPP
#define SYSTEM_MANAGER__SYSTEM_MANAGER__SYSTEM_STATE_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include "state_interfaces/srv/update_state.hpp"

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
     * 系统任务列表
     */
    enum class Task : uint64_t
    {
        STOP = 0,
        JOY_CONTROL = 1, // 手柄控制任务
    };
    /**
     * 系统预设传感器列表
     */
    enum class Sensor : uint64_t
    {
        NONE = 0,
        USB_RGB_CAMERA = 1, // USB RGB相机
        SCI_RGB_CAMERA,     // SCI RGB相机
    };
    /**
     * 系统预设视觉算法列表
     */
    enum class Vison : uint64_t
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
    /**
     * @brief 错误码 (uint64_t)
     */
    enum class ErrorCode : uint64_t
    {
        NO_ERROR = 0,                 // 没有错误
        STATE_UPDATE_GROUP_NOT_FOUND, // 系统状态更新时，未找到对应的状态分组
        STATE_UPDATE_ID_OVERFLOW,     // 系统状态更新时，状态ID超出范围

    };
}
#endif // SYSTEM_MANAGER__SYSTEM_MANAGER__SYSTEM_STATE_HPP