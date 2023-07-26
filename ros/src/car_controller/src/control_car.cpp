#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "car_controller/CarController.h"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarController>());
  rclcpp::shutdown();
  return 0;
}