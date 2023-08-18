#ifndef SYSTEM_MANAGER__SYSTEM_MANAGER__SYSTEM_MANAGER_HPP
#define SYSTEM_MANAGER__SYSTEM_MANAGER__SYSTEM_MANAGER_HPP
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

#include "system_manager/system_state.hpp"

using I_SS = state_interfaces::msg::SystemState; // 状态接口 系统状态 类型
using S_US = state_interfaces::srv::UpdateState; // 更新状态 服务接口
using TIME_MS = std::chrono::milliseconds;       // 毫秒时间
/**
 * @brief 系统状态管理器
 */
class SystemManager : public rclcpp::Node
{
private:
  const TIME_MS ss_publish_timer_interval = TIME_MS(1000);    // 系统状态发布定时器回调间隔 单位：毫秒
  const std::string ss_publisher_topic_name = "system_state"; // 系统状态发布的话题名称
  const rclcpp::QoS ss_publisher_qos = rclcpp::QoS(10);       // 系统状态发布 QoS

  const std::string update_state_server_name = "__server__update_system_state"; // 系统状态更新的服务名称

public:
  explicit SystemManager(std::string name = "system_manager");

public:
  /**
   * @brief 更新系统状态的时间戳
   */
  void update_time_stamp();
  /**
   * @brief 更新系统状态
   * @param group 系统状态分组
   * @param id 系统状态ID
   * @param state 需要设置的系统状态，true为开启(启用)，false为关闭(禁用)
   */
  uint64_t update_state(uint8_t group, uint8_t id, bool state = true);

private:
  I_SS system_state; // 系统状态

  // 任务状态
  const std::unordered_map<uint8_t, uint64_t *> state_pointer_map{
      {static_cast<uint8_t>(SystemState::StateGroup::TASK), &system_state.task_state},
      {static_cast<uint8_t>(SystemState::StateGroup::SENSOR), &system_state.sensor_state},
      {static_cast<uint8_t>(SystemState::StateGroup::VISION), &system_state.vision_state},
  };

  rclcpp::TimerBase::SharedPtr ss_publish_timer_;        // 系统状态发布定时器
  rclcpp::Publisher<I_SS>::SharedPtr ss_publisher_;      // 系统状态发布者
  rclcpp::Service<S_US>::SharedPtr update_state_server_; // 更新系统状态的服务

  /**
   * @brief 系统状态发布定时器回调函数
   */
  void ss_publish_timer_callback();
  /**
   * @brief 更新系统状态的服务回调函数
   * @param request 更新系统状态的请求
   * @param response 更新系统状态的响应
   */
  void update_system_state_server_callback(
      const std::shared_ptr<S_US::Request> request,
      std::shared_ptr<S_US::Response> response);
};

#endif // SYSTEM_MANAGER__SYSTEM_MANAGER__SYSTEM_MANAGER_HPP