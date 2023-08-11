#include "rclcpp/rclcpp.hpp"

#include <video_streamer/video_reader.h>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoReader>());
    rclcpp::shutdown();
    return 0;
}
