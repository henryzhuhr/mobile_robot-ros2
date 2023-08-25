import struct
import time
import serial
import serial.tools.list_ports
import numpy as np
"""
cu.usbmodem11303
cu.usbserial-1110
tty.usbmodem11303
tty.usbserial-1110
"""


def main():
    serial_port = "/dev/ttyUSB0"
    baudrate = 115200
    ser_base = "/dev/tty"
    ser = None
    if serial_port is None:
        for comport in list(serial.tools.list_ports.comports()):
            port = list(comport)[0]
            print(ser_base, port)
            if len(ser_base) < len(port):
                if ser_base == port[:len(ser_base)]:
                    print("find port:", port)
                    serial_port = port
    ser = serial.Serial(
        serial_port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,  # 数据位
        parity=serial.PARITY_NONE,  # 奇偶校验
        stopbits=serial.STOPBITS_ONE,  # 停止位
        timeout=0.1  # 读超时设置
    )

    if ser is None:
        print("no port found")
        return
    
    STATE_LIST=[
        (0.2,0,0),
        (0,0,0),
        (0,0.1,0),
        (0,0,0),
        (0,0,1.0),
        (0,0,0),
    ]
    state_idx=0
    state_cnt=0

    try:
        while True:
            buffer=set_buffer(*STATE_LIST[state_idx])
            ser.write(buffer)
            print("send:", " ".join(["%02X" % d for d in buffer]))
            time.sleep(0.05)
            state_cnt+=1
            if state_cnt>=10:
                state_cnt=0
                state_idx=(state_idx+1)%len(STATE_LIST)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        

    finally:
        state_cnt=10
        while state_cnt>0:
            state_cnt-=1
        buffer=set_buffer(0,0,0)
        ser.write(buffer)
            
        ser.close()


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

if __name__ == "__main__":
    main()
