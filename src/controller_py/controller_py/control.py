import math
from typing import Union

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from interfaces.msg import (Lanes, RuntimeState)

from .serial_control.serial_control import SerialControl
from .decision_planning.vision_lanes import LanesDetectionDecision
from .utils.image_buffer import ImageViewerBuffer

from .types import SpeedState, ButtonState, JOY_KB_MAP


class ControlState:
    """控制状态机"""

    MODE_MAP = {
        0x00: "STOP",
        0x01: "JOY",
        0x02: "AUTO_LANE",
    }
    STOP = 0x00  # 停止
    JOY = 0x01  # 手柄控制
    AUTO = 0x02  # 自动控制，根据视觉算法结果自动修正车辆位置

    def __init__(self):
        self.runtimestate = RuntimeState()
        self.runtimestate.mode = 0x01  # uint8
        self.runtimestate.modules = 0x00  # uint64
        self.speed = SpeedState()

    def next_mode(self):
        """ 切换至下一个控制模式 """
        """
        8 4 2 1
        _ _ _ _
        """
        if self.runtimestate.mode == 0x02:
            self.runtimestate.mode = 0x01
        else:
            self.runtimestate.mode = self.runtimestate.mode << 1


class AutoController(Node):
    def __init__(self):
        super().__init__("auto_controller")
        self.get_logger().info(
            "\033[01;32mCar Auto Controller Node Started\033[0m")
        # 手柄键位映射文件
        self.declare_parameter(
            "joy_config", "configs/joys/Microsoft-X-Box-360-pad.json")
        joy_config = self.get_parameter(
            "joy_config").get_parameter_value().string_value
        self.joy_kb_map = JOY_KB_MAP(joy_config)
        self.get_logger().info(
            f"\033[01;36mJoy Config:\033[0m {joy_config}"
            "\n"
            f"{self.joy_kb_map}"
        )

        # 初始化消息通信
        self.init_sub_pub()

        # -- 打开串口 --
        self.ser_ctr = SerialControl(
            serial_type="USB",
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

        # 小车的控制状态
        self.control_state = ControlState()
        self.get_logger().info(
            f"\033[01;36mControl Mode Init:\033[0m {ControlState.MODE_MAP[self.control_state.runtimestate.mode]}"
        )

        # 超参数设定
        # 最大偏移角度限制 15度 (转换后为弧度单位)
        self.max_z_offset = 15 * math.pi / 180

        # 校准系数，定义为 图像距离/实际距离 (实际操作的时候，图像长 / 测量图像两端在世界中距离)
        self.calibration_ratio = 1280/60  # 1280px / 60cm
        # 最大横向偏移限制 0.1m (转换后为像素单位)
        self.max_y_offset = 0.1*self.calibration_ratio * 40

        # 设置图像缓冲
        self.image_viewer_buffer = ImageViewerBuffer(maxsize=1)

    def init_sub_pub(self):
        # -- 定时器 (状态显示与状态发送) --
        self.state_timer = self.create_timer(
            0.1,  # 100ms
            self.state_timer_callback,
        )
        self.state_timer

        # -- 定时器 (串口发送) --
        self.serial_sender_timer = self.create_timer(
            0.1,  # 100ms
            self.serial_sender_callback,
        )
        self.serial_sender_timer

        # -- 订阅手柄消息 ( Joy 是 ROS2 内置的节点 ，读取 /dev/input/js0 ) --
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )
        self.joy_subscription  # prevent unused variable warning

        # -- 订阅 [车道线检测] 的消息 --
        self.lane_detetion_subscription = self.create_subscription(
            Lanes,
            "vision_lanes",
            self.lanedet_callback,
            10
        )
        self.lane_detetion_subscription  # prevent unused variable warning
        self.lanes_detection_decision = LanesDetectionDecision()

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

    def state_timer_callback(self):
        """
        状态显示的回调
        """
        pass
        # self.get_logger().info(
        #     f"\033[01;36m[状态]:\033[0m"
        #     f" mode: {self.control_state.runtimestate.mode},"
        #     f" modules: {self.control_state.runtimestate.modules},"
        # )

    def serial_sender_callback(self):
        """
        串口发送的回调
        """
        self.ser_ctr.serial_frame.set_speed("x", self.control_state.speed.x)
        self.ser_ctr.serial_frame.set_speed("y", self.control_state.speed.y)
        self.ser_ctr.serial_frame.set_speed("z", self.control_state.speed.z)
        self.ser_ctr.send_car()

    def joy_callback(self, joy_msg: Joy):
        """ 手柄手动控制的回调 """

        # 按键 select 翻转计数  有线手柄 8 无线手柄 6
        # TODO 优化: 记录上升沿和下降沿为一次高电平触发，而不是翻转计数
        if joy_msg.buttons[self.joy_kb_map.button_select] != self.button_select_state.last_state:
            self.button_select_state.flip_cnt += 1
        self.button_select_state.last_state = joy_msg.buttons[self.joy_kb_map.button_select]

        if self.button_select_state.flip_cnt == 2 * 2:  # 4次翻转，连续按下两次
            self.control_state.next_mode()
            self.button_select_state.flip_cnt = 0
            self.get_logger().info(
                f"\033[01;36mControl Mode Switching:\033[0m"
                f" {self.control_state.MODE_MAP[self.control_state.runtimestate.mode]}"
            )

        # 有线手柄 摇杆(joystick): 美国手
        # js_l_x = self.sign(joy_msg.axes[0])  # 左摇杆 x 轴 航向 course
        # js_l_y = self.sign(joy_msg.axes[1])        # 左摇杆 y 轴 升降 lift  (禁用)
        # js_r_x = self.sign(joy_msg.axes[2])  # 右摇杆 x 轴 前后 forwardback
        # js_r_y = self.sign(joy_msg.axes[3])  # 右摇杆 y 轴 左右 leftright

        # 无线手柄 摇杆(joystick): 美国手
        # 左摇杆 x 轴 航向 course
        js_l_x = self.sign(joy_msg.axes[self.joy_kb_map.axis_stick_left__LR])
        # 左摇杆 y 轴 升降 lift  (禁用)
        js_l_y = self.sign(joy_msg.axes[self.joy_kb_map.axis_stick_left__UD])
        # 右摇杆 x 轴 前后 forwardback
        js_r_x = self.sign(joy_msg.axes[self.joy_kb_map.axis_stick_right_UD])
        # 右摇杆 y 轴 左右 leftright
        js_r_y = self.sign(joy_msg.axes[self.joy_kb_map.axis_stick_right_LR])

        # -----------

        scale = 0.2  # 手动控制时建议设置一个系数，防止速度过快

        if (self.control_state.runtimestate.mode & self.control_state.JOY) == self.control_state.JOY:
            self.control_state.speed.x = js_r_x[0] * js_r_x[1] * 1.3 * scale
            self.control_state.speed.y = js_r_y[0] * js_r_y[1] * 1.3 * scale
            self.control_state.speed.z = js_l_x[0] * js_l_x[1] * 3.14 * scale
            # self.get_logger().info(
            #     f"AXES:{list(joy_msg.axes)} "
            #     f"BUTTON:{list(joy_msg.buttons)} "
            #     # f"XYZ:({js_r_x}, {js_r_y}, {js_l_x})"
            #     # f"SEND:[{self.control_state.speed.x}, {self.control_state.speed.y}, { self.control_state.speed.z}]"
            # )

    def lanedet_callback(self, msg: Lanes):
        """ 车道线自动控制的回调 """

        # 修正车道线偏移
        y_offset = msg.y_offset
        z_offset = msg.z_offset

        y_sign, y_val = self.sign(y_offset, 50, 1280)
        z_sign, z_val = self.sign(z_offset, 0.5, 3.14)

        """
        根据 y_offset 和 z_offset 计算出 x/y/z 轴的速度
        1. 优先考虑 z 的误差，先把车摆正
        2. 然后考虑 y 的误差，调整车的位置
        误差调整的时候，减小 x 的速度
        """

        if (self.control_state.runtimestate.mode & self.control_state.AUTO) == self.control_state.AUTO:
            self.control_state.speed.x = 0.1  # 设置最大速度 0.1
            self.control_state.speed.y = -1*y_sign / 1280*50
            self.control_state.speed.z = -0.005*z_sign
            # self.get_logger().info(
            #     f"车道线偏移 ({y_offset}, {z_offset:.3f}))"
            #     f"  校准速度 ({self.control_state.speed.y :.3f}, {self.control_state.speed.z:.3f})")

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


def main(args=None):
    rclpy.init(args=args)
    node = AutoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
