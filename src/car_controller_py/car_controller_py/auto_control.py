import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy

from interfaces.msg import Lanes

from .serial_control.serial_control import SerialControl


class AutoController(Node):
    class ControlLockState:
        """ 控制锁状态 """
        FREE = 0   # 没有线程控制
        USED = 1

    __lock_state = ControlLockState.FREE
    __last_lock_state = ControlLockState.FREE

    def __init__(self):
        super().__init__('auto_controller')
        self.get_logger().info("\033[01;32mCar Auto Controller Node Started\033[0m")

        # 打开串口
        self.ser_ctr = SerialControl()
        if self.ser_ctr.serial_port is None:
            self.get_logger().fatal(f"\033[01;31mOpen serial failed\033[0m ")
        else:
            self.get_logger().info(f"\033[01;32mSuccessfully open serial: "
                                   f"\033[0m{self.ser_ctr.serial_port.name}")

        # -- 订阅手柄消息 ( Joy 是 ROS2 内置的节点 ，读取 /dev/input/js0 )
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

        # -- 定时器 --
        self.reset_speed_timer = self.create_timer(0.1, self.reset_speed_timer_callback)
        self.reset_speed_timer

        # 控制锁，防止多个节点同时控制
        # 只有当手柄控制时，才能修改，优先级最高
        self.__lock_state = self.ControlLockState.FREE
        self.__last_lock_state = self.ControlLockState.FREE

    def joy_callback(self, joy_msg: Joy):
        """
        # ROS 内置手柄控制文档 http://wiki.ros.org/joy
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
            axes=[-0.0, -0.0, -0.0, -0.0, 0.0, 0.0], 
            buttons=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )
        ```
        """

        axes_value = joy_msg.axes
        buttons_value = joy_msg.buttons
        left_L_flag = buttons_value[4] # 左上方 L 按键

        # 摇杆(joystick): 美国手
        js_left__x = self.sign(axes_value[0]) # 左摇杆 x 轴 航向 course
        js_left__y = self.sign(axes_value[1]) # 左摇杆 y 轴 升降 lift  (禁用)
        js_right_x = self.sign(axes_value[2]) # 右摇杆 x 轴 前后 forwardback
        js_right_y = self.sign(axes_value[3]) # 右摇杆 y 轴 左右 leftright

        speed_scale = 0.2 # 手动控制时建议设置一个系数，防止速度过快

        # self.get_logger().info(f"{left_L_flag} {axes_value}")# TODO: 可以注释掉
        # if left_L_flag == 1:# 按键按下才能控制
        if js_right_y[1] != 0 or js_right_x[1] != 0 or js_left__x[1] != 0:
            # x/y 轴速度 需要给 -1 ，原因未知，根据实际车调试控制
            set_y = self.ser_ctr.serial_frame.set_speed("x", -1 * js_right_y[0] * js_right_y[1] * speed_scale)
            set_x = self.ser_ctr.serial_frame.set_speed("y", -1 * js_right_x[0] * js_right_x[1] * speed_scale)
            set_theta = self.ser_ctr.serial_frame.set_speed("theta", js_left__x[0] * js_left__x[1] * speed_scale)

            self.__lock_state = self.ControlLockState.USED
            frame, frame_list = self.ser_ctr.send_car()

            # 重置状态锁 记录上一次状态
            self.__lock_state = self.ControlLockState.FREE
            self.__last_lock_state = self.ControlLockState.USED

    def lanedet_callback(self, lane_result_msg: Lanes):
        """ 车道线自动控制的回调 """

        if self.__lock_state == self.ControlLockState.FREE:
            
            # self.get_logger().info(f"lane_result: {lane_result_msg}")

            y_offset = lane_result_msg.y_offset # y_offset
            z_offset = lane_result_msg.z_offset # z_offset

            self.get_logger().info(f"y_offset: {y_offset} ,  z_offset: {z_offset}")

            speed_scale = 0.2 # 手动控制时建议设置一个系数，防止速度过快

            # x/y 轴速度 需要给 -1 ，原因未知，根据实际车调试控制
            set_y = self.ser_ctr.serial_frame.set_speed("x", -1 * y_offset/5000. * speed_scale)
            set_x = self.ser_ctr.serial_frame.set_speed("y", -1 * speed_scale)
            set_theta = self.ser_ctr.serial_frame.set_speed("theta", z_offset * speed_scale)

            self.__lock_state = self.ControlLockState.USED
            frame, frame_list = self.ser_ctr.send_car()

            # 重置状态锁 记录上一次状态
            self.__lock_state = self.ControlLockState.FREE
            self.__last_lock_state = self.ControlLockState.USED

    def reset_speed_timer_callback(self):
        """ 重置速度 """
        if self.__lock_state == self.ControlLockState.FREE and self.__last_lock_state == self.ControlLockState.FREE:
            self.ser_ctr.serial_frame.set_speed("x", 0)
            self.ser_ctr.serial_frame.set_speed("y", 0)
            self.ser_ctr.serial_frame.set_speed("theta", 0)
            frame, frame_list = self.ser_ctr.send_car()
        self.__last_lock_state = self.ControlLockState.FREE

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