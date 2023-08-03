#include "car_controller/CarController.h"

CarController::CarController() : Node("car_controller")
{
    lane_detetion_subscription = this->create_subscription<std_msgs::msg::String>(
        "lane_result", // 订阅话题
        10,
        std::bind(&CarController::lanedet_callback, this, std::placeholders::_1) // 回调函数
    );
}

void CarController::lanedet_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}