#ifndef MANAGER__SYSTEM_MANAGER__SYSTEM_MANAGER_HPP
#define MANAGER__SYSTEM_MANAGER__SYSTEM_MANAGER_HPP
#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

#include "system_state/state.hpp"
#include "system_state/topics.hpp"
#include "system_state/services.hpp"

namespace manager
{
  /**
   * @brief 系统状态管理器
   */
  class SystemManager : public rclcpp::Node
  {
  private:
    const std::string system_config = "configs/system_state.json"; // 系统状态配置文件

    void parse_config();

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
    SI_M_SS system_state; // 系统状态

    // 任务状态
    const std::unordered_map<uint8_t, std::pair<uint64_t *, uint64_t *>> state_pointer_map{
        {static_cast<uint8_t>(SystemState::StateGroup::TASK), {&system_state.task_enable, &system_state.task_state}},
        {static_cast<uint8_t>(SystemState::StateGroup::SENSOR), {&system_state.sensor_enable, &system_state.sensor_state}},
        {static_cast<uint8_t>(SystemState::StateGroup::VISION), {&system_state.vision_enable, &system_state.vision_state}},
    };

    /**
     * @brief 系统状态更新服务相关
     */
    rclcpp::Service<SI_S_US>::SharedPtr update_state__services_;                            // 更新系统状态的服务
    const std::string update_state__services_name = SystemState::services::update_state; // 系统状态更新的服务名称
    void update_system_state__services_callback(const std::shared_ptr<SI_S_US::Request> request, std::shared_ptr<SI_S_US::Response> response);

    /**
     * @brief 系统状态发布定时器相关
     */
    const TIME_MS ss_publish_timer_interval = TIME_MS(20);                         // 系统状态发布定时器回调间隔 单位：毫秒
    const std::string ss_publisher_topic_name = SystemState::topics::system_state; // 系统状态发布的话题名称
    const rclcpp::QoS ss_publisher_qos = rclcpp::QoS(10);                          // 系统状态发布 QoS
    rclcpp::TimerBase::SharedPtr ss_publish_timer_;                                // 系统状态发布定时器
    rclcpp::Publisher<SI_M_SS>::SharedPtr ss_publisher_;                              // 系统状态发布者
    void ss_publish_timer_callback();
  };

} // namespace manager
#endif // MANAGER__SYSTEM_MANAGER__SYSTEM_MANAGER_HPP