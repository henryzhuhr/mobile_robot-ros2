
from typing import Union
import rclpy
from rclpy.node import Node
from .types import JOY_KB_MAP
from sensor_msgs.msg import Joy

from .types import JOY_KB_MAP

from state_interfaces.msg import Speed
from system_state import SystemState

class SensorJOY(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(
            "\033[01;32mSensor_JOY Node Started\033[0m")
        # 手柄键位映射文件
        self.declare_parameter(
            "joy_config", "configs/sensors/sensor_joy/key_maps/wire_joy.json")
        joy_config = self.get_parameter(
            "joy_config").get_parameter_value().string_value
        
        self.joy_kb_map = JOY_KB_MAP(joy_config)

        self.get_logger().info(
            f"\033[01;36mJoy Config:\033[0m {joy_config}"
            "\n"
            f"{self.joy_kb_map}"
        )
       
        # -- 订阅手柄消息 ( Joy 是 ROS2 内置的节点 ，读取 /dev/input/js0 ) --
        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )
        self.joy_subscription  # prevent unused variable warning
        
        #--发布手柄消息
        self.joy_publishment = self.create_publisher(
            Speed,SystemState.topics.joy_speed, 10
        )
        self.joy_publishment  # prevent unused variable warning

        self.get_logger().info("\033[01;32mSensor_JOY Node Started\033[0m")

        
    def joy_callback(self, joy_msg: Joy):
        """ 手柄手动控制的回调 """
        # 摇杆(joystick): 美国手
        # 左摇杆 x 轴 航向 course
        js_l_x = joy_msg.axes[self.joy_kb_map.axis_stick_left__LR]
        # 左摇杆 y 轴 升降 lift  (禁用)
        js_l_y = joy_msg.axes[self.joy_kb_map.axis_stick_left__UD]
        # 右摇杆 x 轴 前后 forwardback
        js_r_x = joy_msg.axes[self.joy_kb_map.axis_stick_right_UD]
        # 右摇杆 y 轴 左右 leftright
        js_r_y = joy_msg.axes[self.joy_kb_map.axis_stick_right_LR]

        speed=Speed()

        speed.x=js_r_x
        speed.y=js_r_y
        speed.z=js_l_x

        self.joy_publishment.publish(speed)
        # self.get_logger().info(f'发布手柄消息：{joy_msg.axes} {speed.x, speed.y, speed.z}')


def main(args=None):
    rclpy.init(args=args) 
    node = SensorJOY("sensor_joy_py") 
    rclpy.spin(node) 
    rclpy.shutdown() 
