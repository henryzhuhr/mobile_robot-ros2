import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

from system_state import SystemState
from state_interfaces.srv import UpdateState as SI_S_US

class BaseNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.__update_state_=SI_S_US.Request()
        self.update_state__client_=self.create_client(SI_S_US, SystemState.services.update_state)
        self.__heartbeat_timer_:Timer=None

    def InitNode(self,state_group:int,state_id:int,heartbeat_interval_sec:float=1.0,debug=False)->int:
        """
        初始化节点. 设置节点的状态组和状态ID
        ==================
        Args:
        - `state_group`: 状态组. 查看 `SystemState::StateGroup`
        - `state_id`: 状态ID. 查看 `SystemState::Task`、`SystemState::Sensor`、`SystemState::Vison`
        - `heartbeat_interval`: 心跳包发送间隔，默认为 10s. 心跳检测是为了检测节点是否在线，因此不应该太频繁
        - `debug`: 是否为调试模式. 调试模式下不会发送心跳包
        """
        if state_group>SystemState.GROUP_NUM_MAX:
            return SystemState.ErrorCode.STATE_UPDATE_GROUP_OVERFLOW
        
        if state_id>SystemState.ID_NUM_MAX:
            return SystemState.ErrorCode.STATE_UPDATE_ID_OVERFLOW
        self.__update_state_.state_group=state_group
        self.__update_state_.state_id=state_id
        self.__update_state_.state=SystemState.State.IDLE
        if not debug:
            self.__heartbeat_timer_=self.create_timer(heartbeat_interval_sec, self.__UpdateState)
        return 0

    def __UpdateState(self):
        while not self.update_state__client_.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting...")
            self.get_logger().warn("UpdateState service not available, waiting again...")
        self.update_state__client_.call_async(self.__update_state_)
        response=self.update_state__client_.call(self.__update_state_)
        self.get_logger().info("UpdateState service called. Response: %s" % response)


def main(args=None):
    rclpy.init(args=args)
    node = BaseNode("base_node")
    node.get_logger().info("base_node_py")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()