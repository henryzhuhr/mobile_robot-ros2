import math
import time
from typing import Union
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from interfaces.msg import Lanes

from .serial_control.serial_control import SerialControl


class AutoController(Node):
    def __init__(self):
        super().__init__("auto_controller")
        self.get_logger().info(
            "\033[01;32mCar Auto Controller Node Started\033[0m")

        # -- 打开串口 --
        self.ser_ctr = SerialControl(
            serial_type="ACM",  # ACM 是 stm32 的下载(串口)线
            baudrate=115200,
            frame_len=18-4,
        )
        if self.ser_ctr.serial_port is None:
            self.get_logger().fatal(f"\033[01;31mOpen serial failed\033[0m ")
        else:
            self.get_logger().info(
                f"\033[01;32mSuccessfully open serial: "
                f"\033[0m{self.ser_ctr.serial_port.name}"
            )

        # -- 订阅手柄消息 ( Joy 是 ROS2 内置的节点 ，读取 /dev/input/js0 ) --
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )
        self.joy_subscription  # prevent unused variable warning

        # -- 订阅 [车道线检测] 的消息 --
        self.lane_detetion_subscription = self.create_subscription(
            Lanes,
            "lane_result",
            self.lanedet_callback,
            10,
        )
        self.lane_detetion_subscription  # prevent unused variable warning

        # -- 定时器 (按键状态清理) --
        self.button_select_state = ButtonState()
        self.button_states = [
            self.button_select_state,
        ]
        self.reset_button_state_timer = self.create_timer(
            1,  # 1s 清理一次按键状态，因此，所有的按键连击需要小于 1s
            self.reset_button_state_callback,
        )
        self.reset_button_state_timer

        # 控制模式状态机标识
        self.control_mode = ControlMode.MANUAL
        self.get_logger().info(
            f"\033[01;36mControl Mode Init:\033[0m {ControlMode.MODE_MAP[self.control_mode]}"
        )

        # 超参数设定
        # 最大偏移角度限制 15度 (转换后为弧度单位)
        self.max_z_offset = 15 * math.pi / 180

        # 校准系数，定义为 图像距离/实际距离 (实际操作的时候，图像长 / 测量图像两端在世界中距离)
        self.calibration_ratio = 1280/60  # 1280px / 60cm
        # 最大横向偏移限制 0.1m (转换后为像素单位)
        self.max_y_offset = 0.1*self.calibration_ratio * 40

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

        # 按键 select [8] 翻转计数
        # TODO 优化: 记录上升沿和下降沿为一次高电平触发，而不是翻转计数
        if joy_msg.buttons[8] != self.button_select_state.last_state:
            self.button_select_state.flip_cnt += 1
        self.button_select_state.last_state = joy_msg.buttons[8]

        if self.button_select_state.flip_cnt == 2 * 2:  # 4次翻转，连续按下两次
            self.control_mode = (
                self.control_mode + 1
            ) % ControlMode.MODE_MAP.__len__()
            self.button_select_state.flip_cnt = 0
            # self.get_logger().info(
            #     f"\033[01;36mControl Mode Switching:\033[0m"
            #     f" {ControlMode.MODE_MAP[self.control_mode]}"
            # )

        # 摇杆(joystick): 美国手
        js_l_x = self.sign(joy_msg.axes[0] * 3.14)  # 左摇杆 x 轴 航向 course
        js_l_y = self.sign(joy_msg.axes[1])        # 左摇杆 y 轴 升降 lift  (禁用)
        js_r_x = self.sign(joy_msg.axes[2] * 1.3)  # 右摇杆 x 轴 前后 forwardback
        js_r_y = self.sign(joy_msg.axes[3] * 1.3)  # 右摇杆 y 轴 左右 leftright

        # -----------

        scale = 0.2  # 手动控制时建议设置一个系数，防止速度过快

        if self.control_mode == ControlMode.MANUAL:
            # 20 max
            set_x = self.ser_ctr.set_speed("x", js_r_x[0] * js_r_x[1] * scale)
            set_y = self.ser_ctr.set_speed("y", js_r_y[0] * js_r_y[1] * scale)
            set_z = self.ser_ctr.set_speed("z", js_l_x[0] * js_l_x[1] * scale)

            frame, frame_list = self.ser_ctr.send_car()
            # frame_str=" ".join([f"{i:02X}" for i in frame])# 用于检查发送的字节流
            # self.get_logger().info(
            #     f"AXES:{list(joy_msg.axes)} SEND:[{set_x}, {set_y}, {set_z}]"
            # )

    def lanedet_callback(self, lane_result_msg: Lanes):
        """ 车道线自动控制的回调 """

        y_offset = lane_result_msg.y_offset  # y_offset: 像素单位
        z_offset = lane_result_msg.z_offset  # z_offset: 弧度单位

        y_sign, y_val = self.sign(y_offset, 30, 1280)
        z_sign, z_val = self.sign(z_offset, 0.1, 3.14)

        """
        根据 y_offset 和 z_offset 计算出 x/y/z 轴的速度
        1. 优先考虑 z 的误差，先把车摆正
        2. 然后考虑 y 的误差，调整车的位置
        误差调整的时候，减小 x 的速度
        """
        # x 和 y 相反 (因为车头朝向和图像坐标系相反)
        # 检测速度过慢，导致需要添加缓冲区，防止修正速度过慢
        x_speed = 0.1  # 设置最大速度 0.3
        y_speed = -1*y_sign / 1280*50  
        z_speed = z_sign*-0.005

        # if abs(z_offset) > self.max_z_offset:  # z_offset 弧度控制
        #     z_speed *= 2  # 以一个较大的速度校正
        # elif abs(y_offset) > self.max_y_offset:
        #     y_speed *= 2
        # if z_val > 0.1:
        #     x_speed = 0
        #     y_speed = 0
        # elif y_val > 150:
        #     z_speed = 0.0
        # if y_sign*z_sign<0:z_speed=0

        # if (y_val>20)

        # 设置速度
        set_x = self.ser_ctr.serial_frame.set_speed("x", x_speed)
        set_y = self.ser_ctr.serial_frame.set_speed("y", y_speed)
        set_z = self.ser_ctr.serial_frame.set_speed("z", z_speed)

        if self.control_mode == ControlMode.AUTO:
            frame, frame_list = self.ser_ctr.send_car()
            frame_str = " "  # .join([f"{i:02X}" for i in frame])  # 用于检查发送的字节流
            self.get_logger().info(
                f"[AUTO]{frame_str}"
                f"y_offset:({self.max_y_offset:.4f}) {y_offset:.4f} ,  z_offset:({self.max_z_offset:.4f}) {z_offset:.4f}"
                f" speed:({x_speed}, {y_speed}, {z_speed}))"
                f" send:({set_x}, {set_y}, {set_z}))"

            )
        else:
            # frame, frame_list = self.ser_ctr.send_car(False)
            frame_str = " "  # .join([f"{i:02X}" for i in frame])  # 用于检查发送的字节流
            self.get_logger().info(
                f"[MANUAL]{frame_str}"
                f" [speed] y:{y_offset} z:{z_offset:.6f}"
                f" send:({set_x}, {set_y}, {set_z:.6f})"
                f" y[{y_sign}, {y_val}] z[{z_sign}, {z_val}]"
            )

    def reset_button_state_callback(self):
        """重置按键状态"""
        for i in range(len(self.button_states)):
            self.button_states[i].flip_cnt = 0

    @staticmethod
    def sign(value: float, zero_threshold: Union[float, int] = 1e-2, max_limit: Union[float, int] = 1.0):
        """返回符号和绝对值"""
        value_sign = 1 if value > 0 else -1
        value_abs = value * value_sign

        if value_abs > max_limit:
            value_abs = max_limit

        if value_abs < zero_threshold:  # 判断是否为 0
            value_sign = 0
            value_abs = 0
        return value_sign, value_abs


class ButtonState:
    """按键状态记录"""

    def __init__(self) -> None:
        self.last_state = 0  # 记录上一次状态
        self.flip_cnt = 0  # 状态翻转计数


class ControlMode:
    """控制模式"""

    MODE_MAP = {
        0: "MANUAL",
        1: "AUTO_LANE",
    }
    MANUAL = 0  # 手动控制
    AUTO = 1  # 自动控制，根据视觉算法结果自动修正车辆位置


def main(args=None):
    rclpy.init(args=args)

    node = AutoController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
