"""


"""
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

import serial

from system_state import SystemState
from state_interfaces.msg import Speed
from base_node import BaseNode


DATA_FLAG = (
    (0x81,33,"imu"),
    (0x82,21,"odo"),
)
class MecanumWheelCar_MotionController(BaseNode):
    """
    麦克纳姆轮小车运动控制器 (Mecanum Wheel Car Motion Controller)
    """
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        serial_port = "/dev/ttyACM0"
        baudrate = 115200
        self.ser = serial.Serial(
            serial_port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,  # 数据位
            parity=serial.PARITY_NONE,  # 奇偶校验
            stopbits=serial.STOPBITS_ONE,  # 停止位
            timeout=0.1  # 读超时设置
        )
        
        if self.ser is None:
            self.get_logger().fatal(f"{SystemState.Color.LRED}Open serial({serial_port}) failed!{SystemState.Color.DEFAULT}")
        else:
            self.get_logger().info(f"{SystemState.Color.LGREEN}Open serial({serial_port}) success!{SystemState.Color.DEFAULT}")

        self.speed = Speed()

        
        self.header = [0xAA, 0x55]
        self.tail = 0xFF
        self.MAX_BUFFER_LEN=100
        self.buffer = [0x00]*self.MAX_BUFFER_LEN

        # 串口发送定时器
        write_timer_period_ms = 50# ms
        self.__serial_write_timer = self.create_timer(
            write_timer_period_ms/1000.0, self.__serial_write_timer_callback
        )
        self.__serial_write_timer

        # 串口接收定时器
        read_timer_period_ms = 10 # ms
        self.__serial_read_timer = self.create_timer(
            read_timer_period_ms/1000.0, self.__serial_read_timer_callback
        )
        self.__serial_read_timer
        

        self.set_speed_sub = self.create_subscription(
            Speed,
            SystemState.topics.set_speed,
            self.__set_speed_callback,
            10
        )

        self.joy_speed_sub=self.create_subscription(
            Speed,
            SystemState.topics.joy_speed,
            self.__set_speed_callback,
            10
        )
        

    def __set_speed_callback(self, speed: Speed):
        self.speed.x = speed.x*1.3*0.2
        self.speed.y = speed.y*1.3*0.2
        self.speed.z = speed.z*3.14*0.5
    
    def __serial_read_timer_callback(self):
        try:
            data_all = self.ser.read_all()
            self.ser.flushInput()   # 每次读取完数据后清空串口缓存区
            if data_all is not None and len(data_all) > 0:
                self.parse_seriarl_data(data_all)            
        except Exception as e:
            self.get_logger().warn(f"serial read error: {e}")
        
    
    def __serial_write_timer_callback(self):
        buffer = self.set_buffer(
            self.speed.x, 
            self.speed.y, 
            self.speed.z
            )
        self.ser.write(buffer)
        self.get_logger().info(" ".join(["%02X" % d for d in buffer]))
        

    def force_stop(self):
        cnt=100# ensure stop
        while cnt>0:
            cnt-=1
            buffer = self.set_buffer(0,0,0)
            self.ser.write(buffer)

    @staticmethod
    def set_buffer(x: float, y: float, z: float):
        # float 转化为 小端4 byte
        x_bytes = struct.pack('b', int(x * 100))
        y_bytes = struct.pack('b', int(y * 100))
        z_bytes = struct.pack('f', float(z))
        # print("x =", x, ":", " ".join(["%02X" % d for d in x_bytes]))
        # print("y =", y, ":", " ".join(["%02X" % d for d in y_bytes]))
        # print("z =", z, ":", " ".join(["%02X" % d for d in z_bytes]))
        buffer = [0x00] * 18
        buffer[0] = 0xAA  # header 1
        buffer[1] = 0x55  # header 2
        buffer[2] = 0x11  # 数据类型标记
        buffer[3] = 0x00  # 保留
        buffer[4] = x_bytes[0]
        buffer[5] = y_bytes[0]
        buffer[6] = z_bytes[0]
        buffer[7] = z_bytes[1]
        buffer[8] = z_bytes[2]
        buffer[9] = z_bytes[3]
        buffer[-2] = sum(buffer[2:-2]) & 0xFF  # checksum
        buffer[-1] = 0xFF  # tail
        return buffer
    
    def parse_seriarl_data(self, data_all: list):
        buffer=self.buffer
        state = 0
        cnt_ = 0
        for u8_data in data_all:
            if state == 0 and u8_data == self.header[0]:
                buffer[cnt_] = u8_data
                cnt_ += 1
                state += 1
            elif state == 1 and u8_data == self.header[1]:
                buffer[cnt_] = u8_data
                cnt_ += 1
                state += 1
            elif state == 2:
                frame_len = 0
                for i in range(len(DATA_FLAG)):
                    if u8_data == DATA_FLAG[i][0]:                        
                        frame_len = DATA_FLAG[i][1]
                        data_name = DATA_FLAG[i][2]
                        break
                if frame_len == 0:
                    state = 0
                    cnt_ = 0
                else:
                    buffer[cnt_] = u8_data
                    cnt_ += 1
                    state += 1
            elif state == 3:
                if cnt_ >= len(buffer):
                    cnt_ = 0
                    state = 0
                buffer[cnt_] = u8_data
                cnt_ += 1
                if u8_data == self.tail and cnt_== frame_len:
                    checksum_indx = cnt_ - 2
                    checksum = 0
                    for i in range(2, checksum_indx):
                        checksum += buffer[i]
                    checksum = checksum & 0xFF
                    if checksum == buffer[checksum_indx]:
                        data = buffer[:cnt_]
                        now_time_str = time.strftime(
                            "%Y-%m-%d %H:%M:%S",
                            time.localtime(time.time()))
                        self.get_logger().info(f"[{now_time_str}][收] [{cnt_}/{frame_len}] ({data_name}) " + " ".join(["%02X" % d for d in data]))
                    state = 0
                    cnt_ = 0
            else:
                state = 0
                cnt_ = 0


def main(args=None):
    rclpy.init(args=args)
    node = MecanumWheelCar_MotionController("motion_node")
    init_error=node.InitNode(
        state_group=SystemState.StateGroup.CONTROLLER,
        state_id=SystemState.StateID.Controller.MECANUM_WHEEL_CAR,
        heartbeat_interval_sec=1.0,
        debug=True
    )
    if init_error==0:
        node.get_logger().info("%s[INIT NODE] Successfully init:%s %s"%(
            SystemState.Color.LGREEN, SystemState.Color.DEFAULT,
            node.get_name()
        ))
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
        finally:
            node.force_stop()
            node.destroy_node()
                
    else:
        node.get_logger().fatal("%s[INIT NODE] Init Node Error (%ld)%s. Please Check Group and id in \"InitNode()\""%(
            SystemState.Color.LRED, init_error, SystemState.Color.DEFAULT
        ))
    
    rclpy.shutdown()