#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }
    
    // 与服务节点类似，以下代码行创建节点，然后为该节点创建客户端:
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<custom_interfaces::srv::AddTwoInts>::SharedPtr client =
        node->create_client<custom_interfaces::srv::AddTwoInts>("add_two_ints");

    // 接下来，创建请求。它的结构是由前面提到的 .srv 文件定义的。
    auto request = std::make_shared<custom_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    // while 循环给客户端1秒钟搜索网络中的服务节点。如果找不到，它将继续等待。
    while (!client->wait_for_service(1s))
    {
        // 如果客户端被取消 (例如，通过您在终端中输入 “ctrl + c”)，它将返回一条错误日志消息，说明它被中断了。
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // 然后，客户端发送其请求，节点旋转直到收到响应或失败。

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");

    rclcpp::shutdown();
    return 0;
}