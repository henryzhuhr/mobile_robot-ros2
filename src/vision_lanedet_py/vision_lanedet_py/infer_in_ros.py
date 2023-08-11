import os
from typing import List

cuda_home = os.getenv("CUDA_HOME")
tensorrt_home = os.getenv("TENSORRT_HOME")
assert os.path.exists(cuda_home), f"environment variable \"CUDA_HOME\" not exists"
assert os.path.exists(tensorrt_home), f"environment variable \"TENSORRT_HOME\" not exists"
os.environ["PATH"] = ":".join([os.getenv("PATH"), f"{tensorrt_home}/bin", f"{cuda_home}/bin"])
os.environ["LD_LIBRARY_PATH"] = ":".join([os.getenv("LD_LIBRARY_PATH"), f"{tensorrt_home}/lib", f"{cuda_home}/lib64"])

import rclpy
from rclpy.node import Node
# ROS 专用的图像格式
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import time
import cv2
from .utils.trt_infer import TensorRTInfer, InferResult

from interfaces.msg import Lanes


class LaneDetector(Node):
    def __init__(self) -> None:
        super().__init__("lane_detector")
        self.get_logger().info("\033[01;32mLaneDetector Node Started\033[0m")
        self.declare_parameter("weight_file", f"{os.getcwd()}/weights/ufld-final-x64.engine")
        self.declare_parameter("skip_frame", 5)
        self.declare_parameter("img_h", 720)
        self.declare_parameter("img_w", 1280)

        self.cv_bridge = CvBridge()
        # 订阅视频流
        self.video_reader_sub = self.create_subscription(
            msg_type=Image,
            topic="video_reader",
            callback=self.video_reader_callback,
            qos_profile=10,
        )
        self.video_reader_sub # prevent unused variable warning

        # 发布车道线检测结果
        self.lane_result_pub = self.create_publisher(
            msg_type=Lanes,
            topic="vision_lanes",
            qos_profile=10,
        )

        trtengine_weight = self.get_parameter("weight_file").get_parameter_value().string_value
        self.get_logger().info(f"\033[01;32mLoad TRT Engine\033[0m:  {trtengine_weight}")
        self.model_infer = TensorRTInfer(trtengine_weight)
        self.get_logger().info(f"\033[01;32mLoad TRT Engine Sucessffully\033[0m")

    def video_reader_callback(self, msg: Image) -> None:
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        infer_result: InferResult = self.model_infer.infer(img)

        lane_y_coords = infer_result.lanes_y_coords              # [18]     所有车道线共用一组 y 坐标
        lanes_x_coords = infer_result.lanes_x_coords             # [4, 18]  4 个车道线的 x 坐标
        lanes_x_coords_kl = infer_result.lanes_x_coords_kl       # [4, 18]  卡尔曼滤波后 4 个车道线的 x 坐标
        lane_center_x_coords = infer_result.lane_center_x_coords # [18]     中心车道线的 x 坐标
        forward_direct = infer_result.forward_direct             # [2, 2]   实际前进方向
        predict_direct = infer_result.predict_direct             # [2, 2]   预测前进方向
        y_offset = infer_result.y_offset
        z_offset = infer_result.z_offset

        img = self.model_infer.mark_result(img, infer_result)

        # 发布车道线检测结果
        lane_result_msg = Lanes()
        lane_result_msg.header = msg.header
        # lane_result_msg.img = self.cv_bridge.cv2_to_imgmsg(img)
        lane_result_msg.lanes_y_coords = lane_y_coords.tolist()
        lane_result_msg.lanes_x_coords = lanes_x_coords.flatten().tolist()
        # lane_result_msg.lanes_x_coords_kl = lanes_x_coords_kl.flatten().tolist()
        # lane_result_msg.lane_center_x_coords = lane_center_x_coords.flatten().tolist()
        # lane_result_msg.forward_direct = forward_direct.flatten().tolist()
        # lane_result_msg.predict_direct = predict_direct.flatten().tolist()
        lane_result_msg.y_offset = int(y_offset)
        lane_result_msg.z_offset = float(z_offset)
        
        self.lane_result_pub.publish(lane_result_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LaneDetector()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()