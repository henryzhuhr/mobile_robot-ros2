#ifndef VIDEO_READER_H
#define VIDEO_READER_H
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
class VideoReader : public rclcpp::Node
{
private:
    std::string topic_name = "video_reader";
    cv::VideoCapture video_cap;
    cv::Mat frame;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    std::string parameter_string_;

public:
    VideoReader();
    ~VideoReader();

private:
    void init_video_cap(std::string video_source=0);
    void timer_callback();
    
};

#endif // VIDEO_READER_H