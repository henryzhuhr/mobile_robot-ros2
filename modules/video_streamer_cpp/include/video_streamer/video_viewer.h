#ifndef __VIDEO_VIEWER_H
#define __VIDEO_VIEWER_H

#include <string>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "interfaces/msg/lanes.hpp"
#include "interfaces/msg/image_viewer.hpp"

#include <opencv2/opencv.hpp>

#include "video_streamer/image_buffer.hpp"

typedef struct
{
    const cv::Scalar red = cv::Scalar(0, 0, 255);
    const cv::Scalar green = cv::Scalar(0, 255, 0);
    const cv::Scalar blue = cv::Scalar(255, 0, 0);
    const cv::Scalar yellow = cv::Scalar(0, 255, 255);
    const cv::Scalar white = cv::Scalar(255, 255, 255);
    const cv::Scalar black = cv::Scalar(0, 0, 0);
    const cv::Scalar gray = cv::Scalar(128, 128, 128);
    const cv::Scalar light_gray = cv::Scalar(192, 192, 192);
    const cv::Scalar dark_gray = cv::Scalar(64, 64, 64);
} ColorMaps;

enum ColorMapsEnum
{
    RED,
    GREEN,
    BLUE,
    YELLOW,
    WHITE,
    BLACK,
    GRAY,
    LIGHT_GRAY,
    DARK_GRAY
};

const std::vector<cv::Scalar> COlOR_LIST = {
    cv::Scalar(0, 0, 255),     // red
    cv::Scalar(0, 255, 0),     // green
    cv::Scalar(255, 0, 0),     // blue
    cv::Scalar(0, 255, 255),   // yellow
    cv::Scalar(255, 255, 255), // white
    cv::Scalar(0, 0, 0),       // black
    cv::Scalar(128, 128, 128), // gray
    cv::Scalar(192, 192, 192), // light_gray
    cv::Scalar(64, 64, 64),    // dark_gray
};

class VideoViewer : public rclcpp::Node
{

public:
    VideoViewer();
    ~VideoViewer();

private:
    const int image_buffer_max_size = 5;
    const int image_buffer_start_buffer = 20; // 前 n 张图像不要
    int buffer_start_count = 0;
    ImageBuffer image_buffer;

    rclcpp::TimerBase::SharedPtr video_viewer_timer;
    void video_viewer_timerr_callback();

    const std::string video_viewer_topic_name = "video_reader";
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr video_viewer__subscription;
    void video_viewer_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    const std::string lanedet_topic_name = "vision_lanes";
    rclcpp::Subscription<interfaces::msg::Lanes>::SharedPtr lanedet__subscription;
    void lanedet_callback(const interfaces::msg::Lanes::SharedPtr msg);
};
#endif