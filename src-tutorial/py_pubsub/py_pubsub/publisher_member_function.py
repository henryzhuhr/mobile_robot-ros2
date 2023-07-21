import rclpy
from rclpy.node import Node

# from std_msgs.msg import String # 导入节点用来构建传递给话题的数据的内置string消息类型

from custom_interfaces.msg import Num    # CHANGE：自定义消息

class MinimalPublisher(Node):
    """
    MinimalPublisher 类，它继承于 Node
    """
    def __init__(self):
        # 追踪是类的构造函数的定义。 super().__init__ 调用 Node 类的构造函数，并为其提供节点名称，在本例中为 minimal_publisher 。
        super().__init__('minimal_publisher')

        # 创建一个发布者，它将在 topic 话题上发布 std_msgs/String 消息类型的消息。
        # 该话题的名称是 topic ，消息类型是 std_msgs/String 。
        # “队列大小” 为10。队列大小是必需的QoS (服务质量) 设置，如果订户接收消息的速度不够快，则限制排队消息的数量。
        self.publisher_ = self.create_publisher(
            # String,                              # msg_type
            Num,                              # CHANGE：自定义消息
            'topic',                             # topic:str
            10,                                  # qos_profile: QoSProfile | int,
        )

        # 创建一个计时器，它将每 0.5 秒调用一次 timer_callback() 方法。
        timer_period = 0.5              # seconds
        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,        # callback 回调函数
        )
        self.i = 0

    def timer_callback(self):
        # msg = String()
        msg=Num()                       # CHANGE：自定义消息
        msg.num = self.i                # CHANGE：自定义消息
        # msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info('Publishing: "%s"' % msg.num)# CHANGE：自定义消息
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()