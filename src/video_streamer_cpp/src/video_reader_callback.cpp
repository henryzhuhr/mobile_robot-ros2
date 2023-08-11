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
        // cv::resize(frame, frame, cv::Size(frame.cols, int(frame.rows / 2)));

        cv::Mat _frame = frame(cv::Range(int(frame.rows / 8), int(frame.rows * 7 / 8)), cv::Range(0, frame.cols));
        // cv2::
        // RCLCPP_INFO(this->get_logger(), "Read frame");
        auto message = sensor_msgs::msg::Image();
        message.header.frame_id = "camera";
        message.header.stamp = this->now();
        message.height = _frame.rows;
        message.width = _frame.cols;
        message.encoding = "bgr8";
        message.is_bigendian = false;
        message.step = _frame.cols * 3;
        size_t size = _frame.cols * _frame.rows * 3;
        message.data.resize(size);
        memcpy(&message.data[0], _frame.data, size);

        // 发布图像
        this->image_publisher->publish(message);

        // cv::imshow("video_reader", _frame);
        // cv::waitKey(1);
        // RCLCPP_INFO(this->get_logger(), "发布图像[%d.%d]", message.header.stamp.sec, message.header.stamp.nanosec);
    }
}