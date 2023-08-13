#include "video_streamer/video_viewer.h"

#include <opencv2/opencv.hpp>

void VideoViewer::video_viewer_timerr_callback()
{
    if (this->image_buffer.size() > 3)
    {
        BufferData *buffer_data = this->image_buffer.dequeue();
        if (buffer_data)
        {
            if (((buffer_data->flag & 2) == 2))
            {
                try
                {
                    for (int i = 0; i < 4; i++)
                    {
                        for (int j = 0; j < 18; j++)
                        {
                            // RCLCPP_WARN(this->get_logger(), "point (%d, %d)", buffer_data->lanes.lanes_x_coords[i * 18 + j], buffer_data->lanes.lanes_y_coords[j]);
                            if (buffer_data->lanes.lanes_x_coords[i * 18 + j] != 0)
                            {
                                cv::circle( //
                                    buffer_data->img,
                                    cv::Point(buffer_data->lanes.lanes_x_coords[i * 18 + j],
                                              buffer_data->lanes.lanes_y_coords[j]),
                                    5,
                                    COlOR_LIST[i],
                                    -1);
                            }
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "[无法绘制车道线] %s", e.what());
                }
            }
            // cv::imshow("video_viewer-buffer", buffer_data->img);
            // cv::waitKey(1);
        }
    }
}

void VideoViewer::video_viewer_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), 1);

    BufferData buffer_data;
    buffer_data.header = msg->header;
    buffer_data.img = frame;
    if (buffer_start_count < image_buffer_start_buffer)
    {
        buffer_start_count++;
        return;
    }
    this->image_buffer.enqueue(buffer_data);
}

void VideoViewer::lanedet_callback(const interfaces::msg::Lanes::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "[收到车道线检测结果] msg ");
    BufferData buffer_data;
    buffer_data.header = msg->header;
    buffer_data.lanes = *msg;
    buffer_data.flag = 2;
    this->image_buffer.try_set(buffer_data);
}