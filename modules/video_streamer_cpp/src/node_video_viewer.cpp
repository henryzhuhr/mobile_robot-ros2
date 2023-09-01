#include "rclcpp/rclcpp.hpp"

#include <video_streamer/video_viewer.h>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoViewer>());
    rclcpp::shutdown();
    return 0;
}
