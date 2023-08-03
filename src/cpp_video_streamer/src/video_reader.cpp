#include "rclcpp/rclcpp.hpp"
#include <video_streamer/video_reader.h>

VideoReader::VideoReader() : Node("video_reader")
{
    this->declare_parameter<std::string>("source", "camera");

    this->get_parameter("source", parameter_string_);
    // RCLCPP_INFO(this->get_logger(), "[获取的参数]get params: %s", parameter_string_.c_str());

    this->init_video_cap(parameter_string_);
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
    // this->video_cap.release();
}

void VideoReader::init_video_cap(std::string video_source)
{
    RCLCPP_ERROR(this->get_logger(), "\033[01;32mOpen video stream:\033[0m \"%s\"", video_source.c_str());
    if (video_source == "camera")
    {
        bool is_open_camera = false;
        for (int i = 0; i < 9; i++)
        {
            video_cap.open(i);
            if (video_cap.isOpened())
            {
                is_open_camera = true;
                RCLCPP_INFO(this->get_logger(), "\033[01;32mOpen camera\033[0m %d", i);
                break;
            }
        }
        if (is_open_camera == false)
        {
            throw std::runtime_error("\033[01;31mCould not open camera by id 0-9\033[0m");
        }
        else
        {
            video_cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);  // 宽度
            video_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); // 高度
            video_cap.set(cv::CAP_PROP_FPS, 30);            // 帧率 帧/秒
            video_cap.set(cv::CAP_PROP_BRIGHTNESS, 1);      // 亮度 1
                                                            // video_cap.set(cv::CAP_PROP_CONTRAST, 40);      // 对比度 40
                                                            // video_cap.set(cv::CAP_PROP_SATURATION, 50);    // 饱和度 50
                                                            // video_cap.set(cv::CAP_PROP_HUE, 50);           // 色调 50
                                                            // video_cap.set(cv::CAP_PROP_EXPOSURE, 50);      // 曝光 50
                                                            // video_cap.set(cv::CAP_PROP_FOURCC, 50);        // FOURCC编解码器的4个字符代码。
                                                            // video_cap.set(cv::CAP_PROP_POS_AVI_RATIO, 0);  // 视频文件的相对位置：0-胶片开始，1-胶片结束。
                                                            // video_cap.set(cv::CAP_PROP_CONVERT_RGB, 1);    // 表示图像是否应转换为RGB的布尔标志
                                                            // video_cap.set(cv::CAP_PROP_RECTIFICATION, 1);  // 立体摄像机的整流标志（注意：只有当前支持DC1394 v 2.x后端）
        }
    }
    else
    {
        bool is_open = video_cap.open(video_source);
        if (is_open == false)
        {
            throw std::runtime_error("\033[01;31mCould not open video stream:\033[0m " + video_source);
        }
    }
}

void VideoReader::timer_callback()
{
    video_cap >> frame;
    if (frame.empty())
    {
        // RCLCPP_ERROR(this->get_logger(), "Could not read frame");
        throw std::runtime_error("\033[01;31mCould not read frame\033[0m");
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

    // RCLCPP_INFO(this->get_logger(), "Read frame stamp[%d.%d]", message.header.stamp.sec, message.header.stamp.nanosec);
}