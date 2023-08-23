#include <rclcpp/rclcpp.hpp>

#include "motion_manager/motion_manager.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller::MotionManager>();
    node->TryOpenSerial();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
