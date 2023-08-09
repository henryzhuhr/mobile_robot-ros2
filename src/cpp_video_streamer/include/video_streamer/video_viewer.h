#ifndef VIDEO_VIEWER_H
#define VIDEO_VIEWER_H

#include <string>
#include <queue>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "interfaces/msg/lanes.hpp"

typedef struct
{
    cv::Mat img;
    interfaces::msg::Lanes lanes; // 车道线检测结果
} MarkedImg;


class VideoViewer : public rclcpp::Node
{
private:
    rclcpp::Subscription<interfaces::msg::Lanes>::SharedPtr video_reader__subscription;
    std::queue<MarkedImg> markedimg_queue; // 用于存储接收到的图像以及标签，如果队列满了，则丢弃最早的图像

public:
    VideoViewer(std::string subscribe_topic_name = "lane_result");
    ~VideoViewer();

private:
    void callback(const interfaces::msg::Lanes::SharedPtr msg) const;
};

#endif