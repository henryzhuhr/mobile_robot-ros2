import struct
import time
import serial
import serial.tools.list_ports
import numpy as np
"""
ls /dev/tty* /dev/cu.* 
cu.usbmodem11303
cu.usbserial-1110
tty.usbmodem11303
tty.usbserial-1110
"""

DATA_FLAG = (
    # (0x11, "control", 18),
    (0x81, "imu", 33),
    (0x82, "odometer", 21),
)
STATE_LIST=[
    (0.2,0,0),(0,0,0),
    (-0.2,0,0),(0,0,0),
    (0,0.1,0),(0,0,0),
    (0,-0.1,0),(0,0,0),
    (0,0,1.0),(0,0,0),
    (0,0,-1.0),(0,0,0),
]

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


def main():
    serial_port = "/dev/ttyACM0"
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
        timeout=0.001  # 读超时设置
    )

    if ser is None:
        print("no port found")
        return

    try:
        
        last_time = time.time()

        
        wstate=0
        wcnt_=0

        while True:
            data_all = ser.read_all()
            # print("   ",type(data_all),data_all)
            if data_all is not None and len(data_all) > 0:
                rbuffer = [0] * 500
                rcnt_ = 0
                rstate = 0  # 收到 AA 时，state=1，收到 55 时，state=2
                frame_len = 0
                data_name = ""
                for u8_data in data_all:
                    # u8_data = struct.unpack('B', data)[0]
                    # print(type(u8_data),hex(u8_data), u8_data)

                    if rstate == 0 and u8_data == 0xAA:
                        rbuffer[rcnt_] = u8_data
                        rcnt_ += 1
                        rstate += 1
                    elif rstate == 1 and u8_data == 0x55:
                        rbuffer[rcnt_] = u8_data
                        rcnt_ += 1
                        rstate += 1
                    elif rstate == 2:
                        frame_len = 0
                        for i in range(len(DATA_FLAG)):
                            if u8_data == DATA_FLAG[i][0]:
                                frame_len = DATA_FLAG[i][2]
                                data_name = DATA_FLAG[i][1]
                                break
                        if frame_len == 0:
                            rstate = 0
                            rcnt_ = 0
                        else:
                            rbuffer[rcnt_] = u8_data
                            rcnt_ += 1
                            rstate += 1
                    elif rstate == 3:
                        if rcnt_ >= len(rbuffer):
                            rcnt_ = 0
                            rstate = 0
                        rbuffer[rcnt_] = u8_data
                        rcnt_ += 1
                        if u8_data == 0xFF and rcnt_ == frame_len:
                            # checksum_indx = cnt_ - 2
                            # checksum = 0
                            # for i in range(2, checksum_indx):
                            #     checksum += buffer[i]
                            # checksum = checksum & 0xFF
                            # if checksum == buffer[checksum_indx]:
                            data = rbuffer[:rcnt_]
                            now_time_str = time.strftime(
                                "%Y-%m-%d %H:%M:%S",
                                time.localtime(time.time()))
                            print(f"[{now_time_str}][收] [{rcnt_}/{frame_len}] ({data_name})",
                                    " ".join(["%02X" % d for d in data]))
                            rstate = 0
                            rcnt_ = 0
                            # ser.flushInput()# 每次读取完数据后清空串口缓存区
                    else:
                        rstate = 0
                        rcnt_ = 0

            wbuffer=set_buffer(*STATE_LIST[wstate])
            ser.write(wbuffer)
            time.sleep(0.05)
            wcnt_+=1
            if wcnt_>=10:
                now_time_str = time.strftime(
                            "%Y-%m-%d %H:%M:%S",
                            time.localtime(time.time()))
                # print(f"[{now_time_str}][发] "," ".join(["%02X" % d for d in wbuffer]))
                wcnt_=0
                wstate=(wstate+1)%len(STATE_LIST)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    finally:
        wcnt_=10
        while wcnt_>0:
            wcnt_-=1
        wbuffer=set_buffer(0,0,0)
        ser.write(wbuffer)
            
        ser.close()


if __name__ == "__main__":
    main()
