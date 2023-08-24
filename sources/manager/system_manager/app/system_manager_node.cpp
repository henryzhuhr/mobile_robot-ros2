#include <rclcpp/rclcpp.hpp>

#include "system_manager/system_manager.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemManager>(); // 创建对应节点的共享指针对象
  rclcpp::spin(node);                            // 运行节点，并检测退出信号
  rclcpp::shutdown();
  return 0;
}
