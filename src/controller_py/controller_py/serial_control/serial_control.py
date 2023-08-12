import struct
from typing import List
import serial
from serial import serialposix

from .serial_frame import CarSerialFrame


class SerialControl:

    # 支持的串口类型 tty*
    SUPPORTED_SERIAL_TYPES = [
        "USB",
        "THS",
        "ACM",                 # stm32 的下载(串口)线
    ]

    def __init__(self, serial_type:str=None, baudrate: int = 115200,frame_len:int=18-4):
        if (serial_type is not None) and (serial_type in SerialControl.SUPPORTED_SERIAL_TYPES):
            tye_open_serial = [serial_type]
        else:
            tye_open_serial = SerialControl.SUPPORTED_SERIAL_TYPES
        self.serial_port = None
        for serial_type in tye_open_serial:
            self.serial_port: serialposix.Serial = self.open_serial(
                serial_type, baudrate=baudrate)

        self.serial_frame = CarSerialFrame(frame_len)

    def open_serial(self, serial_type: str, baudrate: int):
        serial_name_base = f"/dev/tty{serial_type}"
        id = 0
        serial_port = None
        while id < 9:
            try:
                serial_port = serial.Serial(
                    port=serial_name_base + str(id),
                    baudrate=baudrate,
                )
                break
            except:
                id += 1
        return serial_port

    def set_speed(self, speed_type: str, speed: float):
        return self.serial_frame.set_speed(speed_type,speed)

    def send_car(self,is_send:bool=True):
        frame_list = self.serial_frame.get_byte_frame()

        # 拼接字节流
        frame = frame_list[0]
        for i in range(1, len(frame_list)):
            frame += frame_list[i]

        # 串口发送数据
        self.serial_port.write(frame)
        return frame, frame_list
