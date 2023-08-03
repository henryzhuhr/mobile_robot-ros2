

#ifndef __CAR_CONTROLLER_H__
#define __CAR_CONTROLLER_H__
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CarController : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_detetion_subscription;

public:
    CarController();

private:
    void lanedet_callback(const std_msgs::msg::String::SharedPtr msg);
};
#endif // __CAR_CONTROLLER_H__