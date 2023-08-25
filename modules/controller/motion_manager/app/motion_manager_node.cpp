#include <rclcpp/rclcpp.hpp>

#include "motion_manager/motion_manager.hpp"
#include "motion_manager/motion_serial.hpp"

#include "serial/serial.h"

using namespace controller;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller::MotionManager>();
    node->StartReadSerial();
    node->EnableReadSerial();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
