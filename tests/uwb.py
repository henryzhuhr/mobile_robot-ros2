import serial
import struct


# set up serial
def uwb_ser_init(port="COM12", baud_rate=115200):
    ser = serial.Serial(port, baud_rate, timeout=1)
    return ser


# close serial
def uwb_ser_close(ser):
    ser.close()


# get valid data
def uwb_get_valid_data(ser, verbose=False):
    valid_data = []
    valid_flag = False
    if verbose:
        print("received origin data = ", end=':')
    try:
        while True:
            # read one byte from serial
            origin_data = ser.read(1)
            # analysis
            unpacked_origin_data = struct.unpack('>' + 'B', origin_data)
            # convert to dec format int
            unpacked_origin_data = unpacked_origin_data[0]

            if verbose:
                hex_origin_data = hex(unpacked_origin_data)
                print(hex_origin_data, end=' ')

            # wait for valid data, exam string:0xAC DA 00 03
            if origin_data and not valid_flag:
                if unpacked_origin_data == 0xAC:
                    origin_data = ser.read(1)  # read one byte from serial
                    unpacked_origin_data = struct.unpack('>' + 'B', origin_data)  # decode
                    unpacked_origin_data = unpacked_origin_data[0]  # convert to dec format int
                    if unpacked_origin_data == 0xDA:
                        origin_data = ser.read(1)
                        unpacked_origin_data = struct.unpack('>' + 'B', origin_data)
                        unpacked_origin_data = unpacked_origin_data[0]
                        if unpacked_origin_data == 0x00:
                            origin_data = ser.read(1)
                            unpacked_origin_data = struct.unpack('>' + 'B', origin_data)
                            unpacked_origin_data = unpacked_origin_data[0]
                            if unpacked_origin_data == 0x03:  # Output ranging and positioning information
                                valid_flag = True

            # start to store valid bytes
            if origin_data and valid_flag:
                for _ in range(38):
                    origin_data = ser.read(1)  # read one byte from serial
                    unpacked_origin_data = struct.unpack('>' + 'B', origin_data)  # decode
                    unpacked_origin_data = unpacked_origin_data[0]  # convert to dec format int
                    valid_data.append(unpacked_origin_data)

                # print(valid_data)

                # 解算与A，B，C基站的距离
                DA = struct.unpack_from('>H', bytes(valid_data[0:2]))[0]
                DB = struct.unpack_from('>H', bytes(valid_data[2:4]))[0]
                DC = struct.unpack_from('>H', bytes(valid_data[4:6]))[0]
                DD = struct.unpack_from('>H', bytes(valid_data[6:8]))[0]

                # 解算X，Y，Z轴坐标
                X = struct.unpack_from('>h', bytes(valid_data[-6:-4]))[0]
                Y = struct.unpack_from('>h', bytes(valid_data[-4:-2]))[0]
                Z = struct.unpack_from('>h', bytes(valid_data[-2:]))[0]

                # print("DA/B/C = ", DA, " ", DB, " ", DC, end='\t')
                # print("X/Y/Z = ", X, " ", Y, " ", Z)

                valid_data.clear()

                return X, Y, Z, DA, DB, DC, DD
    except KeyboardInterrupt:
        pass


def uwb_get_loc(ser, ave_cap=10, filter_size=2):
    xyz = [[], [], []]
    xyz_ave = [0, 0, 0]
    # store some pixels to analysis
    while all(len(lst) < ave_cap for lst in xyz):
        x, y, z, _, _, _, _ = uwb_get_valid_data(ser)
        xyz[0].append(x)
        xyz[1].append(y)
        xyz[2].append(z)

    for idx in range(3):
        xyz[idx] = sorted(xyz[idx])
        xyz[idx] = xyz[idx][filter_size:-1 * filter_size]  # filter the max 2 and the min 2
        xyz_ave[idx] = sum(xyz[idx]) / len(xyz[idx])  # calc the mean
        xyz[idx].clear()

    # print("x/y/z_ave = ", x_ave, " ", y_ave, " ", z_ave)
    return xyz_ave


SER = uwb_ser_init()
while True:
    XYZ_AVE = uwb_get_loc(SER)
    print("(X, Y, Z) = ", XYZ_AVE)
