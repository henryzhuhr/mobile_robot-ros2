
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

from system_state import SystemState
from base_node import BaseNode

class TemplateNode(BaseNode):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        
def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode("template_node")
    init_error=node.InitNode(
        state_group=SystemState.StateGroup.TASK,
        state_id=SystemState.StateID.Task.JOY_CONTROL,
    )
    if init_error==0:
        rclpy.spin(node)
        node.get_logger().info("%s[INIT NODE] Successfully init:%s %s"%(
            SystemState.Color.LGREEN, SystemState.Color.DEFAULT,
            node.get_name()
        ))
    else:
        node.get_logger().fatal("%s[INIT NODE] Init Node Error (%ld)%s. Please Check Group and id in \"InitNode()\""%(
            SystemState.Color.LRED, init_error, SystemState.Color.DEFAULT
        ))
    node.destroy_node()
    rclpy.shutdown()