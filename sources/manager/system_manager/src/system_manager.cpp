#include <rclcpp/rclcpp.hpp>

#include "system_manager/system_manager.hpp"
#include "system_manager/system_state.hpp"

#define set_bit(x, y) x |= (1 << y)     // 将 X 的第 Y 位 置1
#define reset_bit(x, y) x &= ~(1 << y)  // 将 X 的第 Y 位 清0
#define reverse_bit(x, y) x ^= (1 << y) // 将 X 的第 Y 位 取反

SystemManager::SystemManager(std::string name) : Node(name)
{
    this->ss_publish_timer_ = this->create_wall_timer(
        this->ss_publish_timer_interval,                           // period
        std::bind(&SystemManager::ss_publish_timer_callback, this) // callback
    );
    this->ss_publisher_ = this->create_publisher<I_SS>(
        this->ss_publisher_topic_name, // 话题名称
        this->ss_publisher_qos         // 服务质量Qos
    );

    this->update_state_server_ = this->create_service<S_US>(
        this->update_state_server_name,
        std::bind(&SystemManager::update_system_state_server_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "[INIT] Start Node: %s", name.c_str());
    RCLCPP_INFO(this->get_logger(), "[STATE] system task          : %ld", system_state.task_state);
    RCLCPP_INFO(this->get_logger(), "[STATE] system task(enable)  : %ld", system_state.task_enable);
    RCLCPP_INFO(this->get_logger(), "[STATE] system sensor        : %ld", system_state.sensor_state);
    RCLCPP_INFO(this->get_logger(), "[STATE] system sensor(enable): %ld", system_state.sensor_enable);
    RCLCPP_INFO(this->get_logger(), "[STATE] system sensor(enable): %ld", system_state.sensor_enable);
}

void SystemManager::update_time_stamp()
{
    this->system_state.header.stamp = this->now();
}

uint64_t SystemManager::update_state(uint8_t group, uint8_t id, bool state)
{
    auto iter = this->state_pointer_map.find(group);
    if (iter == this->state_pointer_map.end())
    {
        return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_GROUP_NOT_FOUND);
    }
    else
    {
        if (id > SystemState::ID_NUM_MAX)
        {
            return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_ID_OVERFLOW);
        }
        else
        {
            if (state)
            {
                *(iter->second) |= (1 << id);
            }
            else
            {
                *(iter->second) &= ~(1 << id);
            }
            return static_cast<uint64_t>(SystemState::ErrorCode::NO_ERROR);
        }
    }
}

void SystemManager::ss_publish_timer_callback()
{
    this->update_time_stamp();
    this->ss_publisher_->publish(this->system_state);
#define Debug
#ifdef Debug
    RCLCPP_INFO(this->get_logger(),
                "%s[STATE]%s %sheader.stamp%s: %10d,%-10d %stask%s:%lx(%lx) %ssensor%s:%lx(%lx) %s",
                SystemState::Color::LGREEN,  //
                SystemState::Color::DEFAULT, //
                //
                SystemState::Color::CYAN,                //
                SystemState::Color::DEFAULT,             //
                this->system_state.header.stamp.sec,     //
                this->system_state.header.stamp.nanosec, //
                //
                SystemState::Color::CYAN,       //
                SystemState::Color::DEFAULT,    //
                this->system_state.task_state,  //
                this->system_state.task_enable, //
                //
                SystemState::Color::CYAN,         //
                SystemState::Color::DEFAULT,      //
                this->system_state.sensor_state,  //
                this->system_state.sensor_enable, //
                SystemState::Color::DEFAULT       //

    );
#endif
}

void SystemManager::update_system_state_server_callback(
    const std::shared_ptr<S_US::Request> request,
    std::shared_ptr<S_US::Response> response)
{
    response->error_code = this->update_state(request->group,
                                              request->id,
                                              request->state);
}