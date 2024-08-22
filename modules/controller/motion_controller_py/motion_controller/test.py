
import rclpy
from rclpy.node import Node

from system_state import SystemState
from state_interfaces.msg import Speed

STATE_LIST=[
    (0.4,0,0),(0,0,0),
    (-0.4,0,0),(0,0,0),
    
    # (0,0.1,0),(0,0,0),
    # (0,-0.1,0),(0,0,0), 

    # (0,0,1),(0,0,0), 
    # (0,0,-1),(0,0,0),
]
 
class SetSpeedPublisher(Node):
    def __init__(self) -> None:
        super().__init__("SetSpeedPublisher")
        self.speed_pub = self.create_publisher(
            Speed,
            SystemState.topics.set_speed,
            10
        )
        self.timer_=self.create_timer(0.1, self.__timer_callback)
        self.state=0
        self.state_cnt=0
    
    def __timer_callback(self):
        speed=Speed()
        x,y,z=STATE_LIST[self.state]
        speed.x=float(x)
        speed.y=float(y)
        speed.z=float(z)
        self.state_cnt+=1
        if self.state_cnt>=10:
            self.state_cnt=0
            self.state=(self.state+1)%len(STATE_LIST)
            
        self.speed_pub.publish(speed)
        # self.get_logger().info(f"set speed: ({speed.x},{speed.y},{speed.z}) " )
   
def main(args=None):
    rclpy.init(args=args)
    node = SetSpeedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()