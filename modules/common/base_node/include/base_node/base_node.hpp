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
    std::shared_ptr<SI_S_US::Request> update_state_;

    rclcpp::Client<SI_S_US>::SharedPtr update_state__client_; // 状态更新客户端
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;            // 心跳包定时器
    /**
     * @brief 心跳包发送，向系统服务端发送节点的状态
     */
    void UpdateState();

protected:
    

public:
    explicit BaseNode(const std::string &node_name = "base_node");
    ~BaseNode();

    /**
     * @brief 初始化节点. 设置节点的状态组和状态ID
     * @param state_group 状态组. 查看 `SystemState::StateGroup`
     * @param state_id 状态ID. 查看 `SystemState::Task`、`SystemState::Sensor`、`SystemState::Vison`
     * @param heartbeat_interval 心跳包发送间隔，默认为 10s. 心跳检测是为了检测节点是否在线，因此不应该太频繁
     */
    uint64_t InitNode(uint8_t state_group,
                      uint8_t state_id,
                      TIME_S heartbeat_interval_sec = TIME_S(10)); // 初始化节点
};

#endif // COMMON__BASE_NODE__BASE_NODE_HPP