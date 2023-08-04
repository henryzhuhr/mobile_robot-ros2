import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from interfaces.msg import Lanes

from .serial_control.serial_control import SerialControl


class ButtonState:
    def __init__(self) -> None:
        self.last_state = 0 # 记录上一次状态
        self.flip_cnt = 0   # 状态翻转计数


class AutoController(Node):
    class ControlMode:
        """
        控制模式
        """
        MODE_MAP = {
            0: "MANUAL",
            1: "AUTO_LANE",
        }
        MANUAL = 0    # 手动控制
        AUTO_LANE = 1 # 基于车道线的自动控制

    def __init__(self):
        super().__init__('auto_controller')
        self.get_logger().info("\033[01;32mCar Auto Controller Node Started\033[0m")

        # -- 打开串口 --
        self.ser_ctr = SerialControl()
        if self.ser_ctr.serial_port is None:
            self.get_logger().fatal(f"\033[01;31mOpen serial failed\033[0m ")
        else:
            self.get_logger().info(f"\033[01;32mSuccessfully open serial: "
                                   f"\033[0m{self.ser_ctr.serial_port.name}")

        # -- 订阅手柄消息 ( Joy 是 ROS2 内置的节点 ，读取 /dev/input/js0 ) --
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.joy_subscription # prevent unused variable warning

        # -- 订阅 [车道线检测] 的消息 --
        self.lane_detetion_subscription = self.create_subscription(
            Lanes,
            'lane_result',
            self.lanedet_callback,
            10,
        )
        self.lane_detetion_subscription # prevent unused variable warning

        # -- 定时器 (按键状态清理) --
        self.button_select_state = ButtonState()
        self.button_states = [
            self.button_select_state,
        ]
        self.reset_button_state_timer = self.create_timer(
            1,                                             # 1s 清理一次按键状态，因此，所有的按键连击需要小于 1s
            self.reset_button_state_callback,
        )
        self.reset_button_state_timer

        # 控制模式状态机标识
        self.control_mode = self.ControlMode.MANUAL
        self.get_logger().info(f"\033[01;36mControl Mode Init:\033[0m {self.ControlMode.MODE_MAP[self.control_mode]}")

    def joy_callback(self, joy_msg: Joy):
        """
        手柄手动控制的回调

        ROS 内置手柄控制文档 http://wiki.ros.org/joy

        Joy 消息结构
        ```python
        sensor_msgs.msg.Joy(
            header=std_msgs.msg.Header(
                stamp=builtin_interfaces.msg.Time(
                    sec=1690353442, 
                    nanosec=770217921
                ), 
                frame_id='joy'
            ), 
            axes=[
                -0.0,   # 左摇杆 x 轴   : 左+ 右-
                -0.0,   # 左摇杆 y 轴   : 上+ 下-
                -0.0,   # 右摇杆 y 轴   : 上+ 下-
                -0.0,   # 右摇杆 x 轴   : 左+ 右-
                0.0,    # 左侧方向键 x 轴: 左+ 右-
                0.0     # 左侧方向键 y 轴: 上+ 下-
                ], 
            buttons=[
                0,      #  0 右侧按键 1
                0,      #  1 右侧按键 2
                0,      #  2 右侧按键 3
                0,      #  3 右侧按键 4
                0,      #  4 左上侧按键 1
                0,      #  5 右上侧按键 1
                0,      #  6 左上侧按键 2
                0,      #  7 右上侧按键 2
                0,      #  8 select
                0,      #  9 start
                0,      # 10 左摇杆按键
                0       # 11 右摇杆按键
                ]
        )
        ```
        """

        # 摇杆(joystick): 美国手
        js_left__x = self.sign(joy_msg.axes[0]) # 左摇杆 x 轴 航向 course
        js_left__y = self.sign(joy_msg.axes[1]) # 左摇杆 y 轴 升降 lift  (禁用)
        js_right_x = self.sign(joy_msg.axes[2]) # 右摇杆 x 轴 前后 forwardback
        js_right_y = self.sign(joy_msg.axes[3]) # 右摇杆 y 轴 左右 leftright

        # 按键 select [8] 翻转计数
        # TODO 优化: 记录上升沿和下降沿为一次高电平触发，而不是翻转计数
        if joy_msg.buttons[8] != self.button_select_state.last_state:
            self.button_select_state.flip_cnt += 1
        self.button_select_state.last_state = joy_msg.buttons[8]

        if self.button_select_state.flip_cnt == 2 * 2:             # 4次翻转，连续按下两次
            self.control_mode = (self.control_mode + 1) % self.ControlMode.MODE_MAP.__len__()
            self.button_select_state.flip_cnt = 0
            self.get_logger().info(
                f"\033[01;36mControl Mode Switching:\033[0m"
                f" {self.ControlMode.MODE_MAP[self.control_mode]}"
            )

        # -----------

        speed_scale = 0.2 # 手动控制时建议设置一个系数，防止速度过快

        if self.control_mode == self.ControlMode.MANUAL:
            # x/y 轴速度 需要给 -1 ，原因未知，根据实际车调试控制
            set_y = self.ser_ctr.serial_frame.set_speed("x", -1 * js_right_y[0] * js_right_y[1] * speed_scale)
            set_x = self.ser_ctr.serial_frame.set_speed("y", -1 * js_right_x[0] * js_right_x[1] * speed_scale)
            set_theta = self.ser_ctr.serial_frame.set_speed("theta", js_left__x[0] * js_left__x[1] * speed_scale)

            frame, frame_list = self.ser_ctr.send_car()
            self.get_logger().info(f"axes: {joy_msg.axes} send:({set_x}, {set_y}, {set_theta}))")

    def lanedet_callback(self, lane_result_msg: Lanes):
        """ 车道线自动控制的回调 """

        if self.control_mode == self.ControlMode.AUTO_LANE:

            y_offset = lane_result_msg.y_offset # y_offset
            z_offset = lane_result_msg.z_offset # z_offset

            speed_scale = 0.2 # 手动控制时建议设置一个系数，防止速度过快

            # x/y 轴速度 需要给 -1 ，原因未知，根据实际车调试控制
            # TODO
            set_y = self.ser_ctr.serial_frame.set_speed("x", 1 * 0.003 * y_offset * speed_scale)
            set_x = self.ser_ctr.serial_frame.set_speed("y", -1 * 0.1)
            set_theta = self.ser_ctr.serial_frame.set_speed("theta", -1 * 0.3 * z_offset * speed_scale)
            frame, frame_list = self.ser_ctr.send_car()
            self.get_logger(
            ).info(f"y_offset: {y_offset} ,  z_offset: {z_offset} send:({set_x}, {set_y}, {set_theta}))")

    def reset_button_state_callback(self):
        """ 重置按键状态 """
        for i in range(len(self.button_states)):
            self.button_states[i].flip_cnt = 0

    @staticmethod
    def sign(value: float):
        """ 返回符号和绝对值 """
        value_sign = 1 if value > 0 else -1
        value_abs = value * value_sign
        if value_abs > 1:
            value_abs = 1

        if value_abs < 1e-2: # 判断是否为 0
            value_sign = 0
            value_abs = 0
        return value_sign, value_abs


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