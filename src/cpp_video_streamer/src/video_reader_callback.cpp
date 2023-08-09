#include "rclcpp/rclcpp.hpp"
#include <video_streamer/video_reader.h>

void VideoReader::timer_callback()
{
    video_cap >> frame;
    if (frame.empty())
    {
        // RCLCPP_ERROR(this->get_logger(), "Could not read frame");
        throw std::runtime_error("\033[01;31mCould not read frame\033[0m");
    }
    else
    {
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "[当前参数] is_save_frame = %s", this->is_save_frame);
#endif
        if (this->is_save_frame)
        {
            std::string file_name = parameters.save_path + "/frame-" + std::to_string(this->now().nanoseconds()) + ".jpg";
            cv::imwrite(file_name, frame);
        }
        // RCLCPP_INFO(this->get_logger(), "Read frame");
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

        // 发布图像
        this->image_publisher->publish(message);
    }

    // RCLCPP_INFO(this->get_logger(), "Read frame stamp[%d.%d]", message.header.stamp.sec, message.header.stamp.nanosec);
}