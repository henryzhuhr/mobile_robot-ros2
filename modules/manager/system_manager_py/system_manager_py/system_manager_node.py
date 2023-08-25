#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SystemManager(Node):
    def __init__(self,node_name="system_manager"):
        super().__init__(node_name)
        self.get_logger().info("节点已启动：%s!" % node_name)
        
def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = SystemManager()  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
