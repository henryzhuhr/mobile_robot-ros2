#ifndef VIDEO_VIEWER_H
#define VIDEO_VIEWER_H

#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

class VideoViewer : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

public:
    VideoViewer(std::string subscribe_topic_name = "video_reader");
    ~VideoViewer(); // 需要加上 {} 或者在实现里写，否则报错 undefined reference to `vtable for VideoViewer'

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) const;
};

#endif