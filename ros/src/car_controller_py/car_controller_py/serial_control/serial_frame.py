from abc import ABC
import struct
from tabnanny import check


class SerialFrame(ABC):
    """串口(UART)数据帧格式:
    - 继承后检查是否需要修改帧头/尾部
    """
    def __init__(self, data_len: int = 11):
        self.header_1 = 0xAA
        self.header_2 = 0x55
        self.end = 0xFF

        self.data_len = data_len
        self.data_frame = [0x00] * self.data_len

    # def get_frame_list(self):
    #     return [
    #         self.header_1,        # 帧头
    #         self.header_2,        # 帧头
    #         *self.data_frame,     # 数据位
    #         sum(self.data_frame), # 校验和
    #         self.end,             # 帧尾
    #     ]


"""


## 串口数据帧设计

运动控制
| 数据位 | 数据说明     |
| :----- | ------------ |
| 0      | 帧头1 `0xAA` |
| 1      | 帧头2 `0x55` |
| 2      | 运动状态控制 |
| 3      | 保留         |
| 4      | x 运动速度   |
| 5      | y 运动速度   |
| 6      | z 运动速度   |
| 7      | 保留         |
| 8      | 保留         |
| 9      | 保留         |
| 10     | 校验和       |
| 11     | 帧尾 `0xFF`  |

速度 int 类型   -127～128 刚好就是 1.3
"""


class CarSerialFrame(SerialFrame):
    """车辆控制数据帧部分封装"""
    class DATA_BIT_ID:
        """数据位下标"""
        HEADER_1 = 0
        HEADER_2 = 1
        DATAFRAME_START = 2
        SPEED_X = 4
        SPEED_Y = 5
        SPEED_THETA = 6

    def __init__(self, data_len: int = 11) -> None:
        super().__init__(data_len)

    def set_speed(self, speed_type: str, speed: float):
        # 提取两位小数部分传输 12.[23]4 -> 23
        # if speed_type == "x" or speed_type == "y":
        speed = int(speed * 100)
        if speed_type == "x":
            idx = self.DATA_BIT_ID.SPEED_X
        elif speed_type == "y":
            idx = self.DATA_BIT_ID.SPEED_Y
        elif speed_type == "theta":
            idx = self.DATA_BIT_ID.SPEED_THETA
        else:
            return None

        self.data_frame[idx - self.DATA_BIT_ID.DATAFRAME_START] = speed
        return speed

    def get_byte_frame(self):
        header1 = struct.pack("B", 0xAA)
        header2 = struct.pack("B", 0x55)
        data_frame_bytes = [struct.pack("b", d) for d in self.data_frame]
        check_sum = struct.pack("B", sum(self.data_frame) & 0xFF)
        end = struct.pack("B", 0xFF)
        return [header1, header2, *data_frame_bytes, check_sum, end]
