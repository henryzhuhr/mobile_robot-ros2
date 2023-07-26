import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vision_lanedet_interfaces.msg import LaneResult

from .serial_control.serial_control import SerialControl



class ManualController(Node):
    def __init__(self):
        super().__init__('manual-car_controller')
        self.get_logger().info("\033[01;32mCar Manual Controller Node Started\033[0m")



def main(args=None):
    rclpy.init(args=args)

    node = ManualController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()