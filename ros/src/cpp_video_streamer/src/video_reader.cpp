#include "rclcpp/rclcpp.hpp"

#include <video_streamer/video_reader.h>

VideoReader::VideoReader() : Node("video_reader")
{
    this->init_video_cap();
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        this->topic_name, // topic_name
        10                // qos
    );
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / 30),         // period   : callback perid in milliseconds
        std::bind(&VideoReader::timer_callback, this) // callback : User-defined callback function.
    );
}
VideoReader::~VideoReader()
{
    this->video_cap.release();
}

void VideoReader::init_video_cap()
{
    video_cap.open(0);
    if (!video_cap.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    }
}

void VideoReader::timer_callback()
{
    video_cap >> frame;
    if (frame.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not read frame");
        return;
    }
    auto message = sensor_msgs::msg::Image();
    message.header.frame_id = "camera";
    message.header.stamp = this->now();
    message.height = frame.rows;
    message.width = frame.cols;
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = frame.cols * 3;
    size_t size = frame.cols * frame.rows * 3;
    message.data.resize(size);
    memcpy(&message.data[0], frame.data, size);
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Read frame stamp[%d.%d]", message.header.stamp.sec, message.header.stamp.nanosec);
}