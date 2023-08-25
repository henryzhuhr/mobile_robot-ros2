import struct
import time
import serial
import serial.tools.list_ports
import numpy as np


def set_buffer(x: float, y: float, z: float):
    # float 转化为 小端4 byte
    x_bytes = struct.pack('b', int(x * 100))
    y_bytes = struct.pack('b', int(y * 100))
    z_bytes = struct.pack('f', float(z))
    # print("x =", x, ":", " ".join(["%02X" % d for d in x_bytes]))
    # print("y =", y, ":", " ".join(["%02X" % d for d in y_bytes]))
    # print("z =", z, ":", " ".join(["%02X" % d for d in z_bytes]))
    data_send = [0x00] * 18
    data_send[0] = 0xAA  # header 1
    data_send[1] = 0x55  # header 2
    data_send[2] = 0x11  # 数据类型标记
    data_send[3] = 0x00  # 保留
    data_send[4] = x_bytes[0]
    data_send[5] = y_bytes[0]
    data_send[6] = z_bytes[0]
    data_send[7] = z_bytes[1]
    data_send[8] = z_bytes[2]
    data_send[9] = z_bytes[3]
    data_send[-2] = sum(data_send[2:-2]) & 0xFF  # checksum
    data_send[-1] = 0xFF  # tail
    return data_send


data_send = set_buffer(0.1, 0, 0)
data_send_hex = " ".join(["%02X" % d for d in data_send])
print(data_send_hex)

data_send = set_buffer(0, 0.1, 0)
data_send_hex = " ".join(["%02X" % d for d in data_send])
print(data_send_hex)

data_send = set_buffer(0, 0, 0.1)
data_send_hex = " ".join(["%02X" % d for d in data_send])
print(data_send_hex)
