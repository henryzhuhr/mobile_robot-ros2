import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from .serial_control.serial_control import SerialControl


class ManualController(Node):
    def __init__(self):
        super().__init__('manual_car_controller')
        self.get_logger().info("\033[01;32mCar Manual Controller Node Started\033[0m")

        # -- 订阅手柄消息
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_subscription # prevent unused variable warning

        # -- 打开串口
        # self.serial_controller = SerialControl()
        # if self.serial_controller.serial_port is None:
        #     self.get_logger().fatal(f"\033[01;31mOpen serial failed\033[0m ")
        # else:
        #     self.get_logger().info(
        #         f"\033[01;32mSuccessfully open serial: "
        #         f"\033[0m{self.serial_controller.serial_port.name}"
        #     )

    @staticmethod
    def sign(value: float):
        """ 返回符号和绝对值 """
        value_sign = 1 if value > 0 else -1
        value_abs = value * value_sign

        if value_abs < 1e-5: # 判断是否为 0
            value_sign = 0
            value_abs = 0
        return value_sign, value_abs

    def joy_callback(self, joy_msg: Joy):
        """
        sensor_msgs.msg.Joy(
            header=std_msgs.msg.Header(
                stamp=builtin_interfaces.msg.Time(
                    sec=1690353442, 
                    nanosec=770217921
                ), 
                frame_id='joy'
            ), 
            axes=[-0.0, -0.0, -0.0, -0.0, 0.0, 0.0], 
            buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )
        """
        js_course_map = {1: "左", 0: "停", -1: "右"}
        js_forwardback_map = {1: "前", 0: "停", -1: "后"}
        js_leftright_map = {1: "左", 0: "停", -1: "右"}
        axes_value = joy_msg.axes
        # 摇杆(joystick): 美国手
        js_left__x = self.sign(axes_value[0]) # 左摇杆 x 轴 航向 course
        js_left__y = self.sign(axes_value[1]) # 左摇杆 y 轴 升降 lift       (禁用)
        js_right_x = self.sign(axes_value[2]) # 右摇杆 x 轴 前后 forwardback
        js_right_y = self.sign(axes_value[3]) # 右摇杆 y 轴 左右 leftright

        # joy_msg.header
        self.get_logger().info(
            f"摇杆量  "
            f"航向({js_course_map[js_left__x[0]]}): {js_left__x[1]:.5f}  "
            f"前后({js_forwardback_map[js_right_x[0]]}): {js_right_x[1]:.5f}  "
            f"左右({js_leftright_map[js_right_y[0]]}): {js_right_y[1]:.5f}  "
        )


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