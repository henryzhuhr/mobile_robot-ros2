#include "base_node/base_node.hpp"
#include "system_state/services.hpp" // 获取全局定义的服务名称

BaseNode::BaseNode(const std::string &node_name) : Node(node_name)
{
    this->update_state__client_ = this->create_client<SI_S_US>(SystemState::services::update_state);
    this->update_state_ = std::make_shared<SI_S_US::Request>();
}

BaseNode::~BaseNode()
{
}

uint64_t BaseNode::InitNode(uint8_t state_group, uint8_t state_id, TIME_S heartbeat_interval_sec)
{

    if (state_group > SystemState::GROUP_NUM_MAX)
        return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_GROUP_OVERFLOW);
    if (state_id > SystemState::ID_NUM_MAX)
        return static_cast<uint64_t>(SystemState::ErrorCode::STATE_UPDATE_ID_OVERFLOW);
    this->update_state_->state_group = state_group;
    this->update_state_->state_id = state_id;
    this->update_state_->state = static_cast<uint8_t>(SystemState::State::IDLE);
    this->heartbeat_timer_ = this->create_wall_timer(heartbeat_interval_sec, std::bind(&BaseNode::UpdateState, this));
    return 0;
}
void BaseNode::UpdateState()
{
    // Wait for the server level to go online
    while (!this->update_state__client_->wait_for_service(std::chrono::seconds(1)))
    {
        // Detect the state of rclcpp while waiting
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service[%s]...", SystemState::services::update_state.c_str());
        }
        RCLCPP_WARN(this->get_logger(), "Waiting for the server[%s] to go online", SystemState::services::update_state.c_str());
    }

    // Send an asynchronous request, then wait to return, and call the callback function when returning
    auto result_future = this->update_state__client_->async_send_request(this->update_state_);
    auto response = result_future.get();
    // RCLCPP_INFO(this->get_logger(), "错误码 %ld", response->error_code);
    
    if (response->error_code != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Update state error: %ld", response->error_code);
    }
    else
    {
        // RCLCPP_INFO(this->get_logger(), "Update state success");
    }
}
