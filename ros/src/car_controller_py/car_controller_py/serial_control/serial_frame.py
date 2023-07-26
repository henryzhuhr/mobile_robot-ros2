from abc import ABC
from typing import List


class SerialFrame(ABC):
    """串口(UART)数据帧格式:
    - 继承后检查是否需要修改帧头/尾部
    """
    def __init__(self, data_len: int = 11):
        self.header_1 = bytes([0xAA])
        self.header_2 = bytes([0x55])
        self.end = bytes([0xFF])

        self.data_len = data_len
        self.data_frame: List[bytes] = [bytes([0x00])] * self.data_len

    @staticmethod
    def int_to_bytes(val: int, byte_len: int = 1):
        return val.to_bytes(byte_len, byteorder='little', signed=False)

    def uchar_checksum(self):
        checksum = 0
        for i in range(self.data_len):
            checksum += int.from_bytes(self.data_frame[i], 'little', signed=False)
        checksum &= 0xFF # 强制截断
        return self.int_to_bytes(checksum, 1)

    def get_frame_list(self):
        return [
            self.header_1,         # 帧头
            self.header_2,         # 帧头
            *self.data_frame,      # 数据位
            self.uchar_checksum(), # 校验和
            self.end,              # 帧尾
        ]

    def get_frame(self):
        frame_bytes = b''
        frame_list = self.get_frame_list()
        for byte in frame_list:
            frame_bytes += byte
        return frame_bytes


class CarSerialFrame(SerialFrame):
    """车辆控制数据帧部分封装"""
    class DATA_BIT_ID:
        """数据位下标"""
        HEADER_1 = 0
        HEADER_2 = 1
        DATAFRAME_START = 2 # 数据位起始下标
        ACTION = 2          # 特殊动作标志位
        SPEED_X = 3
        SPEED_Y = 4
        SPEED_THETA = 5
        TURN_FLAG = 6       # 直行或左偏右偏 标志位 映射到底层位速度的 正负

    class ACTIONS:
        """车辆控制动作枚举"""
        TURN_LEFT_90 = bytes([0x00])
        TURN_RIGHT_90 = bytes([0x01])
        STOP = bytes([0x02])
        # HOLD = bytes([0x02])

    class TURN_FLAGS:
        """车辆控制转向枚举"""
        STRAIGHT = bytes([0x00])
        TURN_LEFT = bytes([0x01])
        TURN_RIGHT = bytes([0x02])

    def __init__(self, data_len: int = 11) -> None:
        super().__init__(data_len)
        self.data_frame[self.DATA_BIT_ID.ACTION - self.DATA_BIT_ID.DATAFRAME_START] = bytes([0x03])

    def set_action(self, action: bytes):
        self.data_frame[self.DATA_BIT_ID.ACTION - self.DATA_BIT_ID.DATAFRAME_START] = action

    def set_speed_x(self, speed: float):
        speed = int(speed * 100) % 100 # 提取两位小数部分传输 12.[23]4 -> 23
        bytes_data = self.int_to_bytes(speed)
        self.data_frame[self.DATA_BIT_ID.SPEED_X - self.DATA_BIT_ID.DATAFRAME_START] = bytes_data
        return bytes_data

    def set_speed_y(self, speed: float):
        speed = int(speed * 100) % 100 # 提取两位小数部分传输 12.[23]4 -> 23
        bytes_data = self.int_to_bytes(speed)
        self.data_frame[self.DATA_BIT_ID.SPEED_Y - self.DATA_BIT_ID.DATAFRAME_START] = bytes_data
        return bytes_data

    def set_speed_theta(self, speed: float):
        bytes_data = self.int_to_bytes(speed)
        self.data_frame[self.DATA_BIT_ID.SPEED_THETA - self.DATA_BIT_ID.DATAFRAME_START] = bytes_data
        return bytes_data

    def set_turn_flag(self, turn_flag: bytes):
        self.data_frame[self.DATA_BIT_ID.TURN_FLAG - self.DATA_BIT_ID.DATAFRAME_START] = turn_flag