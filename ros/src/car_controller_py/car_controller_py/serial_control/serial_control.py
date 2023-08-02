import struct
from typing import List
import serial
from serial import serialposix

from .serial_frame import CarSerialFrame


class SerialControl:
    def __init__(self, serial_type="USB"):
        assert serial_type in [
            "USB",
            "THS",
            "ACM",              # stm32 的下载线
        ], "serial_type must be 'USB' or 'THS'"
        self.serial_port: serialposix.Serial = self.open_serial(serial_type)
        self.serial_frame = CarSerialFrame(11)

    def open_serial(self, serial_type="USB"):
        """
        ] could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0
        
        把自己的用户加入到dialout组
        sudo usermod -aG dialout ${USER}	//user替换为自己的用户名
        reboot							//必须要重启一下才会生效
        """

        serial_name_base = f"/dev/tty{serial_type}"
        id = 0
        serial_port = None
        while id < 18:
            try:
                # 根据实际情况修改串口号和波特率
                serial_port = serial.Serial(
                    port=serial_name_base + str(id),
                    baudrate=115200,
                )
                break
            except:
                id += 1
        return serial_port

    def send_car(self):
        frame_list = self.serial_frame.get_byte_frame()
        # frame = struct.pack("b" * len(frame_list), *frame_list)
        # frame="".join([d for d in frame_list])
        frame=frame_list[0]
        for i in range(1,len(frame_list)):
            frame+=frame_list[i]
        self.serial_port.write(frame)
        return frame, frame_list
