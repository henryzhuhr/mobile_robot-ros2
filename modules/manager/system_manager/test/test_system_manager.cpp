#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

#include "system_state/services.hpp"
#include "system_manager/system_manager.hpp"

class SystemManager_Tester : public rclcpp::Node
{
private:
    bool work_permit; // 该节点是否工作许可标识
    // 声明客户端
    rclcpp::Client<SI_S_US>::SharedPtr client_;

    void result_callback_(rclcpp::Client<SI_S_US>::SharedFuture result_future)
    {
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "错误码 %ld", response->error_code);
    }

public:
    SystemManager_Tester(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", node_name.c_str());
        client_ = this->create_client<SI_S_US>(SystemState::services::update_state);
    }
    ~SystemManager_Tester()
    {
        this->update_system_state(                                   //
            static_cast<uint8_t>(SystemState::StateGroup::SENSOR),   //
            static_cast<uint8_t>(SystemState::StateID::Sensor::JOY), //
            false                                                    //
        );
        RCLCPP_INFO(this->get_logger(), "节点已关闭：%s.", this->get_name());
    };

    void update_system_state(uint8_t group, uint8_t id, bool state = true)
    {

        // 1.等待服务端上线
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            // 等待时检测rclcpp的状态
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service[%s]...", SystemState::services::update_state.c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the server[%s] to go online", SystemState::services::update_state.c_str());
        }

        // 2.构造请求的
        auto request = std::make_shared<SI_S_US::Request>();
        request->state_group = group;
        request->state_id = id;
        request->state = state;

        // 3.发送异步请求，然后等待返回，返回时调用回调函数
        client_->async_send_request(
            request,
            std::bind(
                &SystemManager_Tester::async_response_callback_,
                this,
                std::placeholders::_1));
    };
    void async_response_callback_(
        rclcpp::Client<SI_S_US>::SharedFuture async_response)
    {
        auto response = async_response.get();
        RCLCPP_INFO(this->get_logger(), "异步响应: %ld", response->error_code);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SystemManager_Tester>("test_system_manager");

    // 节点启动时，向系统状态管理器注册
    node->update_system_state(                                   //
        static_cast<uint8_t>(SystemState::StateGroup::SENSOR),   //
        static_cast<uint8_t>(SystemState::StateID::Sensor::JOY), //
        true                                                     //
    );
    rclcpp::spin(node);
    // TODO: 在节点意外退出时，应该向系统状态管理器注销，但是不知道为什么无法调用析构函数

    rclcpp::shutdown();
    return 0;
}
