#include "video_streamer/video_viewer.h"

VideoViewer::VideoViewer(std::string subscribe_topic_name) : Node("video_viewer")
{
    video_reader__subscription = this->create_subscription<interfaces::msg::Lanes>(
        subscribe_topic_name,                                          // topic_name – The topic to subscribe on.
        10,                                                            // qos
        std::bind(&VideoViewer::callback, this, std::placeholders::_1) // callback
    );
}
VideoViewer::~VideoViewer()
{
    cv::destroyAllWindows();
}
void VideoViewer::callback(const interfaces::msg::Lanes::SharedPtr message) const
{
    cv::Mat frame = cv::Mat(message->img.height, message->img.width, CV_8UC3, const_cast<unsigned char *>(message->img.data.data()));
    cv::resize(frame, frame, cv::Size(960, 540));
    cv::imshow("video_viewer", frame);
    cv::waitKey(1);
    // RCLCPP_INFO(this->get_logger(), "Viewer frame stamp[%d.%d]", message->header.stamp.sec, message->header.stamp.nanosec);
}
