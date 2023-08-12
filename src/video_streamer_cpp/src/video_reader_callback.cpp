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

        /**
         * 压缩图像传输:
         * - https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CompressedImage.msg
         * - https://blog.csdn.net/weixin_40153532/article/details/105174445
         *
         */
        auto message = sensor_msgs::msg::CompressedImage();
        // message.header.frame_id = "camera";
        // message.header.stamp = this->now();
        // message.height = _frame.rows;
        // message.width = _frame.cols;
        // message.encoding = "bgr8";
        // message.is_bigendian = false;
        // message.step = _frame.cols * 3;
        // size_t size = _frame.cols * _frame.rows * 3;
        // message.data.resize(size);
        // memcpy(&message.data[0], _frame.data, size);
        message.header.frame_id = "camera";
        message.header.stamp = this->now();
        message.format = "jpeg";
        // std::vector<int> compression_params;
        // compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        // compression_params.push_back(80); // 压缩比率 0-100
        cv::imencode(".jpg", _frame, message.data, {cv::IMWRITE_JPEG_QUALITY, this->img_quality});

        // 发布图像
        this->image_publisher->publish(message);

        // cv::imshow("video_reader", _frame);
        // cv::waitKey(1);
        // RCLCPP_INFO(this->get_logger(), "发布图像[%d.%d]", message.header.stamp.sec, message.header.stamp.nanosec);
    }
}