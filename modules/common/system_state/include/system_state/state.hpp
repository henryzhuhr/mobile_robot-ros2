#ifndef COMMON__SYSTEM_STATE__STATE_HPP
#define COMMON__SYSTEM_STATE__STATE_HPP


#include <string>

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
     * 任务状态以独占形式进行管理，即当前如果有任务在执行时，新的任务指令将返回reject状态
     */
    enum class Task : uint64_t
    {
        IDLE = 0,        // 空闲状态
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

}

#endif // COMMON__SYSTEM_STATE__STATE_HPP