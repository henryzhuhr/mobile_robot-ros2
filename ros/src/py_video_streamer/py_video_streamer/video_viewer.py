import rclpy
from rclpy.node import Node

# ROS 专用的图像格式
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

class VideoViewerSubscriber(Node):
    """
    VideoViewerSubscriber 类，它继承于 Node
    """
    def __init__(self):
        
        super().__init__('video_viewer_subscriber')
        self.subscription = self.create_subscription(Image,"video_reader",self.callback,10)
    
        self.cv_bridge = CvBridge()

        self.subscription  # prevent unused variable warning

    def callback(self,msg):
        img:Image=msg
        frame=self.cv_bridge.imgmsg_to_cv2(img)
        cv2.imshow("frame",frame)
        self.get_logger().info("show frame") # CHANGE：自定义消息

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    video_viewer_subscriber = VideoViewerSubscriber()

    rclpy.spin(video_viewer_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    video_viewer_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()