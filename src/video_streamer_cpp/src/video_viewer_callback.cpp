#include "video_streamer/video_viewer.h"

#include <opencv2/opencv.hpp>

void VideoViewer::video_viewer_timerr_callback()
{
    if (this->image_buffer.size() > 4)
    {
        BufferData *buffer_data = this->image_buffer.dequeue();
        if (buffer_data)
        {
            if (((buffer_data->flag & 2) == 2))
            {
                // RCLCPP_INFO(this->get_logger(), "[显示缓存图像] %d [车道线结果] %d %d", //
                //             buffer_data->flag,                                          //
                //             buffer_data->lanes.lanes_x_coords.size(),                   //
                //             buffer_data->lanes.lanes_y_coords.size()                    //
                // );
                // try
                // {
                //     cv::putText(buffer_data->img, "flag:2", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);
                // }
                // catch (const std::exception &e)
                // {
                //     RCLCPP_WARN(this->get_logger(), "[无法绘制文字] %s", e.what());
                // }
                try
                {
                    for (int i = 0; i < 4; i++)
                    {
                        for (int j = 0; j < 18; j++)
                        {
                            // RCLCPP_WARN(this->get_logger(), "point (%d, %d)", buffer_data->lanes.lanes_x_coords[i * 18 + j], buffer_data->lanes.lanes_y_coords[j]);
                            // if (buffer_data->lanes.lanes_x_coords[i * 18 + j] != 0)
                            // {
                            //     cv::circle( //
                            //         buffer_data->img,
                            //         cv::Point(buffer_data->lanes.lanes_x_coords[i * 18 + j],
                            //                   buffer_data->lanes.lanes_y_coords[j]),
                            //         2,
                            //         cv::Scalar(0, 0, 255),
                            //         2);
                            // }
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "[无法绘制车道线] %s", e.what());
                }
            }
            cv::imshow("video_viewer-buffer", buffer_data->img);
            cv::waitKey(1);
        }
    }
}
void VideoViewer::video_viewer_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{

    // RCLCPP_INFO(this->get_logger(), "[收到图像] msg ");
    cv::Mat frame = cv::Mat( //
        msg->height,
        msg->width,
        CV_8UC3,
        const_cast<unsigned char *>(msg->data.data()) //
    );
    // cv::imshow("video_viewer", frame);
    // cv::waitKey(1);
    BufferData buffer_data;
    buffer_data.header = msg->header;
    buffer_data.img = frame;
    if (buffer_start_count < image_buffer_start_buffer)
    {
        buffer_start_count++;
        return;
    }
    this->image_buffer.enqueue(buffer_data);

    // if ((msg->flag & 2) != 0)
    // {
    //     // cv::putText(frame, "flag:2", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);

    //     for (int i = 0; i < 4; i++)
    //     {
    //         for (int j = 0; j < 18; j++)
    //         {
    //             if (msg->lanes_x_coords[i * 18 + j] != 0)
    //             {
    //                 cv::circle( //
    //                     frame,
    //                     cv::Point(msg->lanes_x_coords[i * 18 + j],
    //                               msg->lanes_y_coords[j]),
    //                     2,
    //                     cv::Scalar(0, 0, 255),
    //                     2);
    //             }
    //         }
    //     }
    // }

    // cv::imshow("video_viewer", frame);
    // cv::waitKey(1);
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