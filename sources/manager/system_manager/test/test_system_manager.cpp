#include "state_interfaces/msg/system_state.hpp"
#include "state_interfaces/srv/update_state.hpp"

#include "system_manager/system_manager.hpp"

class SystemManager_Tester : public rclcpp::Node
{
private:
    // 声明客户端
    rclcpp::Client<S_US>::SharedPtr client_;
    const std::string update_state_server_name = "__server__update_system_state"; // 系统状态更新的服务名称

    void result_callback_(rclcpp::Client<S_US>::SharedFuture result_future)
    {
        auto response = result_future.get();
        RCLCPP_INFO(this->get_logger(), "错误码 %ld", response->error_code);
    }

public:
    SystemManager_Tester(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", node_name.c_str());
        client_ = this->create_client<S_US>(this->update_state_server_name);
    }
    ~SystemManager_Tester(){};

    void update_system_state(uint8_t group, uint8_t id, bool state = true)
    {

        // 1.等待服务端上线
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            // 等待时检测rclcpp的状态
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }

        // 2.构造请求的
        auto request = std::make_shared<S_US::Request>();
        request->group = group;
        request->id = id;
        request->state = state;

        // // 3.发送异步请求，然后等待返回，返回时调用回调函数
        client_->async_send_request(request, std::bind(&SystemManager_Tester::result_callback_, this,
                               std::placeholders::_1));
    };
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SystemManager_Tester>("service_client_01");
    node->update_system_state(                                //
        static_cast<uint8_t>(SystemState::StateGroup::TASK),  //
        static_cast<uint8_t>(SystemState::Task::JOY_CONTROL), //
        true                                                  //
    );
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
