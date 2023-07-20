from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):
    def __init__(self):
        # 构造函数初始化名称为 minimal_service 的节点
        super().__init__('minimal_service')

        # 创建一个服务并定义类型、名称和调用。
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        """ 服务调用back的定义接收请求数据, 对其求和, 并将总和作为响应返回 """
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args) # 初始化ROS 2 Python客户端库

    minimal_service = MinimalService() # 实例化 MinimalService 类以创建服务节点

    rclpy.spin(minimal_service) # 实例化 MinimalService 类以创建服务节点

    rclpy.shutdown()


if __name__ == '__main__':
    main()