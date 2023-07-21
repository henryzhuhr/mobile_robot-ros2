#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<custom_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<custom_interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld b: %ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化ROS2 c client客户端库

    // 创建一个名为 add_two_ints_server 的节点
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    // 为该节点创建名为 add_two_ints 的服务，并自动调用y使用 “& add” 方法在网络上发布该服务
    rclcpp::Service<custom_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<custom_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node); // 旋转节点，使服务可用
    rclcpp::shutdown();
}