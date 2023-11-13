#ifndef COMMON__SYSTEM_STATE__TYPES_HPP
#define COMMON__SYSTEM_STATE__TYPES_HPP

#include <chrono>

#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

using SI_M_SS = state_interfaces::msg::SystemState; // 状态接口 系统状态 类型
using SI_S_US = state_interfaces::srv::UpdateState; // 更新状态 服务接口
using TIME_MS = std::chrono::milliseconds;          // 毫秒时间
using TIME_S  = std::chrono::seconds;               // 秒时间

#endif // COMMON__SYSTEM_STATE__TYPES_HPP