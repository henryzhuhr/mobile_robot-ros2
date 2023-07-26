from typing import List
import serial
from serial import serialposix

from .serial_frame import CarSerialFrame



class SerialControl:
    def __init__(self):
        self.serial_port: serialposix.Serial = self.open_serial()
        self.serial_frame = CarSerialFrame(11)

    def open_serial(self):
        """
        ] could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0
        
        把自己的用户加入到dialout组
        sudo usermod -aG dialout ${USER}	//user替换为自己的用户名
        reboot							//必须要重启一下才会生效
        """

        serial_name_base = "/dev/ttyUSB"
        id = 0
        serial_port = None
        while id < 18:
            try:
                serial_port = serial.Serial(serial_name_base + str(id), 115200) # 根据实际情况修改串口号和波特率
                break
            except:
                id += 1
        return serial_port
    def send_car(self):
        frame_bytes= self.serial_frame.get_frame()
        self.serial_port.write(frame_bytes)
        return frame_bytes