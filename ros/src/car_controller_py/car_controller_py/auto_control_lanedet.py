import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vision_lanedet_interfaces.msg import LaneResult

from .serial_control.serial_control import SerialControl


class AutoController(Node):
    def __init__(self):
        super().__init__('auto_controller')
        self.get_logger().info("\033[01;32mCar Auto Controller Node Started\033[0m")

        # -- 订阅 [车道线检测] 的消息 --
        self.lane_detetion_subscription = self.create_subscription(
            LaneResult,
            'lane_result',
            self.lanedet_callback,
            10,
        )
        self.lane_detetion_subscription # prevent unused variable warning

        # 打开串口
        self.serial_controller = SerialControl()
        if self.serial_controller.serial_port is None:
            self.get_logger().fatal(f"\033[01;31mOpen serial failed\033[0m ")
        else:
            self.get_logger().info(
                f"\033[01;32mSuccessfully open serial: "
                f"\033[0m{self.serial_controller.serial_port.name}"
            )

        self.turn_flag = 0
        self.count = 1

    def lanedet_callback(self, lane_result_msg: LaneResult):
        # self.get_logger().info(f"lane_result: {lane_result_msg}")

        slope = lane_result_msg.slope                   # 预测方向的斜率
        distance_bias = lane_result_msg.offset_distance # 偏移距离

        hold = 1
        if distance_bias > 80 or distance_bias < -80 or slope > 15 or slope < -15:
            hold = 0
            if distance_bias < 0:
                # self.turn_flag = 1 # 检测到车向右偏，车应向左转
                self.serial_controller.serial_frame.set_turn_flag(
                    self.serial_controller.serial_frame.TURN_FLAGS.TURN_LEFT
                )
                print("应  左 转\n")
            elif distance_bias > 0:
                # self.turn_flag = 2 # 检测到车向左偏，车应向右转
                self.serial_controller.serial_frame.set_turn_flag(
                    self.serial_controller.serial_frame.TURN_FLAGS.TURN_RIGHT
                )
                print("应  右 转\n")
            else:
                # self.turn_flag = 0 # 车没偏
                self.serial_controller.serial_frame.set_turn_flag(
                    self.serial_controller.serial_frame.TURN_FLAGS.STRAIGHT
                )

        time.sleep(1)
        if hold == 1:
            theta_speed = 0
            # turn_flag = 0
            self.serial_controller.serial_frame.set_turn_flag(self.serial_controller.serial_frame.TURN_FLAGS.STRAIGHT)
        else:
            #theta_speed = abs(pid_controller(distance_bias, slope))
            theta_speed = int(abs(distance_bias // 100))
        self.serial_controller.serial_frame.set_speed_x(0.20)
        self.serial_controller.serial_frame.set_speed_y(0.00)
        self.serial_controller.serial_frame.set_speed_theta(theta_speed)

        # self.get_logger().info(f"slope: {slope}, distance_bias: {distance_bias} theta_speed: {theta_speed}")

        frame = self.serial_controller.send_car()
        self.get_logger().info(f"send frame : {frame}")


def main(args=None):
    rclpy.init(args=args)

    node = AutoController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()