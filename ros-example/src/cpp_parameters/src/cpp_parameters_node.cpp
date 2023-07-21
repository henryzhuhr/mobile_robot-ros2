#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass : public rclcpp::Node
{
private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    ParametersClass() : Node("parameter_node")
    {
        // 创建我们的参数。我们的参数名为 my_parameter ，并被指定为默认值 world
        this->declare_parameter<std::string>("my_parameter", "world");
        // timer_ 初始化，这导致 respond 函数每秒执行一次
        timer_ = this->create_wall_timer(1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
        // respond 函数的第一行从节点获取参数 my_parameter ，并将其存储在 parameter_string_ 中
        this->get_parameter("my_parameter", parameter_string_);
        RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParametersClass>()); // “rclcpp:: spin” 开始处理来自节点的数据。
    rclcpp::shutdown();
    return 0;
}