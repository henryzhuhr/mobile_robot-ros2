#include <rclcpp/rclcpp.hpp>

#include "motion_controller/motion_controller.hpp"
#include "motion_controller/motion_serial.hpp"

#include "serial/serial.h"

using namespace controller;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller::MotionController>();
    node->StartReadSerial();
    node->EnableReadSerial();
    try
    {
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    rclcpp::shutdown();
    return 0;
}
