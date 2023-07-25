import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vision_lanedet_interfaces.msg import LaneResult

from .utils.serial_control import try_open_serial


class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.get_logger().info("\033[01;32mCar Controller Node Started\033[0m")

        # -- 订阅 [车道线检测] 的消息 --
        self.lane_detetion_subscription = self.create_subscription(
            LaneResult,
            'lane_result',
            self.lanedet_callback,
            10,
        )
        self.lane_detetion_subscription # prevent unused variable warning

        # 打开串口
        ser = try_open_serial()
        if ser is None:
            self.get_logger().fatal(f"\033[01;31mOpen serial failed\033[0m ")
        else:
            self.get_logger().info(f"\033[01;32mSuccessfully open serial: \033[0m{ser.name}")
        self.ser = ser

        self.turn_flag = 0
        self.count = 1

    def lanedet_callback(self, lane_result_msg: LaneResult):
        # self.get_logger().info(f"lane_result: {lane_result_msg}")

        slope = lane_result_msg.slope                   # 预测方向的斜率
        distance_bias = lane_result_msg.offset_distance # 偏移距离

        # self.get_logger().info(f"slope: {slope}, offset_distance: {offset_distance}")

        hold = 1
        if distance_bias > 80 or distance_bias < -80 or slope > 15 or slope < -15:
            hold = 0
            if distance_bias < 0:
                self.turn_flag = 1 # 检测到车向右偏，车应向左转
                print("应  左 转\n")
            elif distance_bias > 0:
                self.turn_flag = 2 # 检测到车向左偏，车应向右转
                print("应  右 转\n")
            else:
                self.turn_flag = 0 # 车没偏

        time.sleep(1)
        if hold == 1:
            theta_speed = 0
            turn_flag = 0
        else:
            #theta_speed = abs(pid_controller(distance_bias, slope))
            theta_speed = int(abs(distance_bias // 100))
        # send_thread(x_speed, y_speed, theta_speed, count, turn_flag)
        # count += 1
        # receive_t = threading.Thread(target=receive_data)
        # receive_t.start()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CarController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()