import rclpy
from rclpy.node import Node

# ROS 专用的图像格式
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

class VideoFramePublisher(Node):
    def __init__(self):
        
        super().__init__('video_reader_publisher')
        self.img_publisher = self.create_publisher(Image,"video_reader",10)

        self.video_cap=cv2.VideoCapture(0)
        self.video_cap.set(cv2.CAP_PROP_FPS, 30)
        self.fps=self.video_cap.get(cv2.CAP_PROP_FPS)

        self.cv_bridge = CvBridge()

        # 创建一个计时器，它将每 0.5 秒调用一次 timer_callback() 方法。
        timer_period =1/self.fps              # seconds
        self.timer = self.create_timer(
            timer_period,
            self.callback,        # callback 回调函数
        )

    def callback(self):
        ret, frame = self.video_cap.read() #opencv returned video frame.
        
        #convert to ROS2 Image msg.
        img:Image = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
        img.header.stamp = self.get_clock().now().to_msg()
        # img.header.frame_id = self.camera_name        
        
        #publish
        self.img_publisher.publish(img)
        self.get_logger().info(f'Read Video. fps:{self.fps}' )# CHANGE：自定义消息


def main(args=None):
    rclpy.init(args=args)

    video_reader_publisher = VideoFramePublisher()

    rclpy.spin(video_reader_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_reader_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()