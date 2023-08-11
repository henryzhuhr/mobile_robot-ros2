#include "video_streamer/video_viewer.h"



VideoViewer::VideoViewer() : Node("video_viewer")
{
    RCLCPP_INFO(this->get_logger(), "\033[01;32mVideo Viewer Node Started\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[01;32m  video_viewer_topic_name:%s\033[0m", this->video_viewer_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "\033[01;32m       lanedet_topic_name:%s\033[0m", this->lanedet_topic_name.c_str());
    this->image_buffer = ImageBuffer(this->image_buffer_max_size);
    /**
     * 创建定时器
     */
    this->video_viewer_timer = this->create_wall_timer(
        std::chrono::milliseconds(200),                             // 100ms
        std::bind(&VideoViewer::video_viewer_timerr_callback, this) // callback
    );

    /**
     * 创建订阅者 订阅视频读取节点的图像
     */
    this->video_viewer__subscription = this->create_subscription<sensor_msgs::msg::Image>(
        this->video_viewer_topic_name,                                              // topic_name
        10,                                                                         // qos
        std::bind(&VideoViewer::video_viewer_callback, this, std::placeholders::_1) // callback
    );

    /**
     * 创建订阅者 订阅车道线检测结果
     */
    this->lanedet__subscription = this->create_subscription<interfaces::msg::Lanes>(
        this->lanedet_topic_name,                                              // topic_name
        10,                                                                    // qos
        std::bind(&VideoViewer::lanedet_callback, this, std::placeholders::_1) // callback
    );
}
VideoViewer::~VideoViewer()
{
    cv::destroyAllWindows();
}
