from abc import ABC
import struct
from tabnanny import check
from typing import List


class SerialFrame(ABC):
    """串口(UART)数据帧格式:
    - 继承后检查是否需要修改帧头/尾部
    """

    def __init__(self, data_len: int = 18-2):
        self.header = [struct.pack("B", d) for d in [0xAA, 0x55]]
        self.tail__ = [struct.pack("B", d) for d in [0xFF]]
        self.data_len = data_len
        self.data: List[bytes] = [b'\x00'] * self.data_len

    def get_byte_frame(self):
        check_sum = 0
        for d in self.data:
            check_sum += d[0]
        check_sum = struct.pack("B", check_sum & 0xFF)

        return [*self.header, *self.data, check_sum, * self.tail__]


class CarSerialFrame(SerialFrame):
    """
    ## 串口数据帧设计

    运动控制
    | 数据位 | 数据说明     |
    | :----- | ------------ |
    | 0      | 帧头1 `0xAA` |
    | 1      | 帧头2 `0x55` |
    | 2      | 数据类型标记 |
    | 3      | 保留         |
    | 4      | x 运动速度   |
    | 5      | y 运动速度   |
    | 6      | z 运动速度   |
    | 7      | z 运动速度    |
    | 8      | z 运动速度    |
    | 9      | z 运动速度    |
    | 10      | 保留         |
    | 11      | 保留         |
    | 12      | 保留         |
    | 13      | 保留         |
    | 14      | 保留         |
    | 15      | 保留         |
    | 16     | 校验和       |
    | 17     | 帧尾 `0xFF`  |
    """
    class DATA_BIT_ID:
        """数据位下标"""
        HEADER_1 = 0
        HEADER_2 = 1
        DATAFRAME_START = 2
        SPEED_X = 4
        SPEED_Y = 5
        SPEED_Z = 6

    def __init__(self, data_len: int = 12 - 4) -> None:
        super().__init__(data_len)

    def set_speed(self, speed_type: str, speed: float):

        idx_offset = self.DATA_BIT_ID.DATAFRAME_START

        if speed_type == "x":
            speed = self.__limit_int_speed(int(speed * 100))
            idx = self.DATA_BIT_ID.SPEED_X
            self.data[idx - idx_offset] = struct.pack("b", speed)
        elif speed_type == "y":
            speed = self.__limit_int_speed(int(speed * 100))
            idx = self.DATA_BIT_ID.SPEED_Y
            self.data[idx - idx_offset] = struct.pack("b", speed)
        elif speed_type == "z":
            speed = self.__limit_float_speed(speed)
            f_bs = struct.pack("f", speed)  # float bytes
            idx = self.DATA_BIT_ID.SPEED_Z
            self.data[idx + 0 - idx_offset] = struct.pack("B", f_bs[0])
            self.data[idx + 1 - idx_offset] = struct.pack("B", f_bs[1])
            self.data[idx + 2 - idx_offset] = struct.pack("B", f_bs[2])
            self.data[idx + 3 - idx_offset] = struct.pack("B", f_bs[3])
        else:
            speed = None

        return speed

    @staticmethod
    def __limit_int_speed(speed: int, min_: int = -128, max_: int = 127):
        """ 限制 int 速度范围 """
        if speed < -128:
            speed = -128
        if speed > 127:
            speed = 127
        return speed

    @staticmethod
    def __limit_float_speed(speed: float, min_: float = -3.14, max_: float = 3.14):
        """ 限制 int 速度范围 """
        if speed < -3.14:
            speed = -3.14
        if speed > 3.14:
            speed = 3.14
        return speed
