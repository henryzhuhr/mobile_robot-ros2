#include <rclcpp/rclcpp.hpp>

#include "system_manager/system_manager.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manager::SystemManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
