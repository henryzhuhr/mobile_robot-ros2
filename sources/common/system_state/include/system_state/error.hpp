#ifndef SYSTEM_MANAGER__SYSTEM_MANAGER__ERROR_HPP
#define SYSTEM_MANAGER__SYSTEM_MANAGER__ERROR_HPP
#include <rclcpp/rclcpp.hpp>
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

    };
}
#endif // SYSTEM_MANAGER__SYSTEM_MANAGER__ERROR_HPP