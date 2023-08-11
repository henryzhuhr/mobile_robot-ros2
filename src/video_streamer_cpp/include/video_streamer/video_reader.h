#ifndef VIDEO_READER_H
#define VIDEO_READER_H
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

// 读取视频类的参数
typedef struct
{
    int camera_id = 0;                          // 相机id
    std::string soruce = "camera";              // 视频源
    std::string save_path = "tmp/video_viewer"; // 保存路径
    int fps = 30;                               // 帧率
} VideoReaderParams;

class VideoReader : public rclcpp::Node
{

public:
    VideoReader();
    ~VideoReader();

private:
    cv::VideoCapture video_cap;
    cv::Mat frame;
    VideoReaderParams parameters; // 读取视频类的参数
    bool is_save_frame = false;   // 是否保存图像帧信号

private:
    /**
     * @brief 初始化视频采集，打开摄像头，根据参数选择打开摄像头还是视频文件
     */
    void init_video_cap();

    /**
     * @brief 解析参数
     */
    void parse_params();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher; // 图像发布者
    std::string image_publisher_topic_name = "video_reader";

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_frame_subscriber; // 保存图像帧订阅者
    std::string save_frame_subscriber_topic_name = "video_saver";

private: // 回调函数在单独的文件中实现
    /**
     * @brief 定时器回调函数
     */
    void timer_callback();
};

#endif // VIDEO_READER_H