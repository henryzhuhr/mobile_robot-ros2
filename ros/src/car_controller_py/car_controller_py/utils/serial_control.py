import serial


def try_open_serial():
    """
    ] could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0
    
    把自己的用户加入到dialout组
    sudo usermod -aG dialout ${USER}	//user替换为自己的用户名
    reboot							//必须要重启一下才会生效
    """

    serial_name_base = "/dev/ttyUSB"
    id = 0
    ser = None
    while id < 18:
        try:
            ser = serial.Serial(serial_name_base + str(id), 115200) # 根据实际情况修改串口号和波特率
            break
        except:
            id += 1
    return ser

def serial_sender():
    header_1 = bytes([0xAA])
    header_2 = bytes([0x55])
    test = bytes([0x00])
    frame_end = bytes([0xFF])  # 加上帧尾，底层需要识别帧尾FF

    flag = 3  # 临时写为0

    # print("vel.linear_x =  {}\nvel.linear_y =  {}\nvel.linear_z =  {}".format(x_speed, y_speed, theta))