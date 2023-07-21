#include <video_streamer/video_viewer.h>
#include <opencv2/opencv.hpp>

VideoViewer::VideoViewer(std::string subscribe_topic_name) : Node("video_viewer")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subscribe_topic_name,                                          // topic_name – The topic to subscribe on.
        10,                                                            // qos
        std::bind(&VideoViewer::callback, this, std::placeholders::_1) // callback
    );
}
VideoViewer::~VideoViewer()
{
    cv::destroyAllWindows();
}
void VideoViewer::callback(const sensor_msgs::msg::Image::SharedPtr message) const
{
    cv::Mat frame = cv::Mat(message->height, message->width, CV_8UC3, const_cast<unsigned char *>(message->data.data()));
    cv::imshow("video_viewer", frame);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Viewer frame stamp[%d.%d]", message->header.stamp.sec, message->header.stamp.nanosec);
}
