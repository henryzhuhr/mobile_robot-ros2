#include "base_node/base_node.hpp"

BaseNode::BaseNode(const std::string &node_name) : Node(node_name)
{
    this->update_state__client_ = this->create_client<S_US>(SystemState::services::update_state);
}

BaseNode::~BaseNode()
{
}

uint64_t BaseNode::InitNode(uint8_t state_group, uint8_t state_id)
{
    if (state_id > SystemState::ID_NUM_MAX)
    {
        return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_ID_OVERFLOW);
    }
    this->state_group_ = state_group;
    this->state_id_ = state_id;
    return 0;
}
uint64_t BaseNode::UpdateState(uint8_t state)
{
    // // 1.等待服务端上线
    // while (!this->update_state__client_->wait_for_service(std::chrono::seconds(1)))
    // {
    //     // 等待时检测rclcpp的状态
    //     if (!rclcpp::ok())
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service[%s]...", SystemState::services::update_state.c_str());
    //         return SystemState::ErrorCode::ROS_RCLCPP_NOT_INIT;
    //     }
    //     RCLCPP_INFO(this->get_logger(), "Waiting for the server[%s] to go online", SystemState::services::update_state.c_str());
    // }

    // 2.构造请求的
    auto request = std::make_shared<S_US::Request>();
    request->state_group = this->state_group_;
    request->state_id = this->state_id_;
    request->state = state;

    // 3.发送异步请求，然后等待返回，返回时调用回调函数
    auto result_future = this->update_state__client_->async_send_request(request);
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "错误码 %ld", response->error_code);
    return response->error_code;
}
