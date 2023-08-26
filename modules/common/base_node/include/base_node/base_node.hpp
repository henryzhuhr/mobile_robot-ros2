#ifndef COMMON__BASE_NODE__BASE_NODE_HPP
#define COMMON__BASE_NODE__BASE_NODE_HPP

#include <string>

#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

#include "rclcpp/rclcpp.hpp"

#include "system_state/state.hpp"
#include "system_state/types.hpp"

/**
 * @brief 系统中所有节点的基类，继承自 `rclcpp::Node`，提供了一些基础功能
 */
class BaseNode : public rclcpp::Node
{
private:
    uint64_t state_group_ = 0; // 状态组
    uint64_t state_id_ = 0;    // 状态ID

    rclcpp::Client<SI_S_US>::SharedPtr update_state__client_; // 状态更新客户端
public:
    explicit BaseNode(const std::string &node_name = "base_node");
    ~BaseNode();
    /**
     * @brief 初始化节点. 设置节点的状态组和状态ID
     * @param state_group 状态组. 查看 `SystemState::StateGroup`
     * @param state_id 状态ID. 查看 `SystemState::Task`、`SystemState::Sensor`、`SystemState::Vison`
     * @code
     * // 例如，初始化节点为手柄控制节点
     * this->update_system_state(
     *  static_cast<uint8_t>(SystemState::StateGroup::SENSOR),
     *  static_cast<uint8_t>(SystemState::Sensor::JOY)
     * );
     * @endcode
     */
    uint64_t InitNode(uint8_t state_group, uint8_t state_id); // 初始化节点
    /**
     * @brief 更新系统状态. 心跳包发送，用于检测节点是否在线
     * @param state 状态值.
     * @return 错误码. 0表示成功，错误码参考 `SystemState::ErrorCode`
     */
    uint64_t UpdateState(uint8_t state);
};

#endif // COMMON__BASE_NODE__BASE_NODE_HPP