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

DATA_FLAG = (
    (0x11, "control", 18),
    (0x81, "imu", 21),
    (0x82, "odometer", 21),
)


def main():
    serial_port = "/dev/ttyUSB0"
    baudrate = 115200
    ser_base = "/dev/cu.usbmodem"
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

    try:
        buffer = [0] * 500
        cnt_ = 0
        state = 0  # 收到 AA 时，state=1，收到 55 时，state=2
        frame_len = 0
        data_name = ""
        last_time = time.time()

        while True:
            data = ser.read(1)
            if data is not None and len(data) > 0:
                u8_data = struct.unpack('B', data)[0]
                # print(type(u8_data),hex(u8_data), u8_data)

                if state == 0 and u8_data == 0xAA:
                    buffer[cnt_] = u8_data
                    cnt_ += 1
                    state += 1
                elif state == 1 and u8_data == 0x55:
                    buffer[cnt_] = u8_data
                    cnt_ += 1
                    state += 1
                elif state == 2:
                    frame_len = 0
                    for i in range(len(DATA_FLAG)):
                        if u8_data == DATA_FLAG[i][0]:
                            frame_len = DATA_FLAG[i][2]
                            data_name = DATA_FLAG[i][1]
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
                    if u8_data == 0xFF and cnt_ == frame_len:
                        # checksum_indx = cnt_ - 2
                        # checksum = 0
                        # for i in range(2, checksum_indx):
                        #     checksum += buffer[i]
                        # checksum = checksum & 0xFF
                        # if checksum == buffer[checksum_indx]:
                        data = buffer[:cnt_]
                        now_time_str = time.strftime(
                            "%Y-%m-%d %H:%M:%S",
                            time.localtime(time.time()))
                        print(f"[{now_time_str}][收] [{cnt_}/{frame_len}] ({data_name})",
                                " ".join(["%02X" % d for d in data]))
                        state = 0
                        cnt_ = 0
                else:
                    state = 0
                    cnt_ = 0

    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
