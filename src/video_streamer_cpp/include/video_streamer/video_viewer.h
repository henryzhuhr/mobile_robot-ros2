#ifndef __VIDEO_VIEWER_H
#define __VIDEO_VIEWER_H

#include <string>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "interfaces/msg/lanes.hpp"
#include "interfaces/msg/image_viewer.hpp"

#include "video_streamer/image_buffer.hpp"

class VideoViewer : public rclcpp::Node
{

public:
    VideoViewer();
    ~VideoViewer();

private:
    const int image_buffer_max_size = 5;
    const int image_buffer_start_buffer = 50;// 前 n 张图像不要
    int buffer_start_count = 0;
    ImageBuffer image_buffer;

    rclcpp::TimerBase::SharedPtr video_viewer_timer;
    void video_viewer_timerr_callback();

    const std::string video_viewer_topic_name = "video_reader";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_viewer__subscription;
    void video_viewer_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    const std::string lanedet_topic_name = "vision_lanes";
    rclcpp::Subscription<interfaces::msg::Lanes>::SharedPtr lanedet__subscription;
    void lanedet_callback(const interfaces::msg::Lanes::SharedPtr msg);
};
#endif