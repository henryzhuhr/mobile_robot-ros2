#include "rclcpp/rclcpp.hpp"
#include <video_streamer/video_reader.h>

VideoReader::VideoReader() : Node("video_reader")
{

    RCLCPP_INFO(this->get_logger(), "\033[01;32mVideo Reader Node Started\033[0m");
    // time_t now = time(0);
    // tm *ltm = localtime(&now);
    // RCLCPP_INFO(this->get_logger(), "当前时间: %d-%d-%d %d:%d:%d", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

    this->parse_params();   // 解析参数
    this->init_video_cap(); // 初始化摄像头

    /**
     * 创建图像话题发布者
     */
    this->image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        this->image_publisher_topic_name, // topic_name
        10                                // qos
    );
    /**
     * 创建保存图像话题订阅者
     */
    this->save_frame_subscriber = this->create_subscription<std_msgs::msg::Bool>(
        this->save_frame_subscriber_topic_name, // topic_name
        10,                                     // qos
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->is_save_frame = msg->data;
        } // callback
    );
    /**
     * 创建定时器
     * @param[in] period   : 回调周期根据视频读取的帧率来设置
     */
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / this->parameters.fps), // period   : callback perid in milliseconds
        std::bind(&VideoReader::timer_callback, this)           // callback : User-defined callback function.
    );

    // 创建文件夹，用于保存视频帧
    system(("mkdir -p " + parameters.save_path).c_str());
}
VideoReader::~VideoReader()
{
    this->video_cap.release();
}

void VideoReader::parse_params()
{
    std::string parameter_string_;
    // 解析参数 source
    {
        this->declare_parameter<std::string>("source", // 定义视频源参数
                                             "camera"  // 默认为相机
        );
        this->get_parameter("source", parameter_string_);
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "[获取的参数]get params: %s", parameter_string_.c_str());
#endif
        if (parameter_string_.substr(0, 6) == "camera") // 如果参数是camera开头的，就认为是相机
        {
            this->parameters.soruce = "camera";
            if (parameter_string_.size() > 6)
            {
                this->parameters.camera_id = std::stoi(parameter_string_.substr(6, parameter_string_.size() - 6));
                // RCLCPP_INFO(this->get_logger(), "[设置参数] params.camera_ids: %d", this->params.camera_id);
            }
        }
        else
        {
            this->parameters.soruce = parameter_string_;
        }
    }
}

void VideoReader::init_video_cap()
{
    RCLCPP_ERROR(this->get_logger(), "\033[01;32mOpen video stream:\033[0m \"%s\"", this->parameters.soruce.c_str());
    if (this->parameters.soruce == "camera")
    {
        bool is_open_camera = false;
        for (int i = this->parameters.camera_id; i < 9; i++)
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
            /**
             * [video_reader-1] [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (935) open OpenCV | GStreamer warning: Cannot query video position: status=0, value=-1, duration=-1
             * [video_reader-1] [INFO] [1691045168.953248177] [t1]: Open camera 2
             * [video_reader-1] [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (1758) handleMessage OpenCV | GStreamer warning: Embedded video playback halted; module v4l2src0 reported: Internal data stream error.
             * [video_reader-1] [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (515) startPipeline OpenCV | GStreamer warning: unable to start pipeline
             * [video_reader-1] [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (1057) setProperty OpenCV | GStreamer warning: no pipeline
             * [video_reader-1] [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (1057) setProperty OpenCV | GStreamer warning: no pipeline
             * [video_reader-1] [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (1057) setProperty OpenCV | GStreamer warning: no pipeline
             * 这种报错的原因是不难修改分辨率
             */
            // video_cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);  // 宽度
            // video_cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); // 高度
            // video_cap.set(cv::CAP_PROP_FPS, 10);       // 帧率 帧/秒
            // video_cap.set(cv::CAP_PROP_BRIGHTNESS, 1); // 亮度 1
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
        bool is_open = video_cap.open(this->parameters.soruce);
        if (is_open == false)
        {
            throw std::runtime_error("\033[01;31mCould not open video stream:\033[0m " + this->parameters.soruce);
        }
    }
}
