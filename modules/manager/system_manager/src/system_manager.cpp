#include "system_manager/system_manager.hpp"

#include "system_state/services.hpp"

namespace manager
{

    SystemManager::SystemManager(std::string name) : Node(name)
    {

        this->parse_config();

        this->ss_publish_timer_ = this->create_wall_timer(
            this->ss_publish_timer_interval,                           // period
            std::bind(&SystemManager::ss_publish_timer_callback, this) // callback
        );
        this->ss_publisher_ = this->create_publisher<SI_M_SS>(
            this->ss_publisher_topic_name, // 话题名称
            this->ss_publisher_qos         // 服务质量Qos
        );

        this->update_state__services_ = this->create_service<SI_S_US>(
            this->update_state__services_name,
            std::bind(&SystemManager::update_system_state__services_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "[INIT] Start Node: %s", name.c_str());
        RCLCPP_INFO(this->get_logger(), "[STATE] system task  : %ld/%ld", system_state.task_state, system_state.task_enable);
        RCLCPP_INFO(this->get_logger(), "[STATE] system sensor: %ld/%ld", system_state.sensor_state, system_state.sensor_enable);
        RCLCPP_INFO(this->get_logger(), "[STATE] system vision: %ld/%ld", system_state.vision_state, system_state.vision_enable);
#endif
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
                bool enable = (*(iter->second.first) & (1 << id)); // 检查系统是否启用该功能
                if (state && enable)
                {
                    if (group == static_cast<uint8_t>(SystemState::StateGroup::TASK))
                    {
                        // TASK 任务是独占式的，只能有一个任务处于运行状态
                        *(iter->second.second) &= (1 << id);
                    }
                    else
                    {
                        *(iter->second.second) |= (1 << id);
                    }
                }
                else
                {
                    *(iter->second.second) &= ~(1 << id);
                }
                return static_cast<uint64_t>(SystemState::ErrorCode::NO_ERROR);
            }
        }
    }

    void SystemManager::ss_publish_timer_callback()
    {
        this->update_time_stamp();
        this->ss_publisher_->publish(this->system_state);
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(),
                    "%s[STATE]%s %sheader.stamp%s: %10d,%-10d %stask%s:%lx(%lx) %ssensor%s:%lx(%lx) %svision%s:%lx(%lx) %s",
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
                                                      //
                    SystemState::Color::CYAN,         //
                    SystemState::Color::DEFAULT,      //
                    this->system_state.vision_state,  //
                    this->system_state.vision_enable, //
                    SystemState::Color::DEFAULT       //

        );
#endif
    }

    void SystemManager::update_system_state__services_callback(
        const std::shared_ptr<SI_S_US::Request> request,
        std::shared_ptr<SI_S_US::Response> response)
    {
        response->error_code = this->update_state(request->state_group,
                                                  request->state_id,
                                                  request->state);
    }
} // namespace manager
