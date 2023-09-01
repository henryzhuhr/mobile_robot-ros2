import rclpy
from rclpy.node import Node
import serial
import struct

from std_msgs.msg import Header

from interfaces.msg import Location


class SensorUWB(Node):
    def __init__(self):
        super().__init__('sensor_uwb')
        self.get_logger().info(
            "\033[01;32mSensorUWB(UWB modules) Node Started\033[0m")

        # [parameter] serial_port 串口名称/波特率
        self.declare_parameter("serial_port__num", "/dev/ttyUSB1")
        self.declare_parameter("serial_baud_rate", 115200)
        serial_port__num = self.get_parameter(
            "serial_port__num").get_parameter_value().string_value
        serial_baud_rate = self.get_parameter(
            "serial_baud_rate").get_parameter_value().integer_value

        self.ser = serial.Serial(serial_port__num, serial_baud_rate, timeout=1)

        if self.ser is None:
            self.get_logger().fatal(
                f"\033[01;31mOpen serial failed!\033[0m "
                f"({serial_port__num}, baud:{serial_baud_rate})")
        else:
            self.get_logger().info(
                f"\033[01;32mOpen serial success!\033[0m "
                f"({serial_port__num}, baud:{serial_baud_rate})")

        # -- 定时器 (读取UWB数据) --
        self.uwb_sampler_timer = self.create_timer(
            1/1000,  # 1 ms
            self.uwb_sampler_timer_callback,
        )
        self.uwb_sampler_timer

        # 发布uwb 定位结果结果
        self.uwb_location_pub = self.create_publisher(
            msg_type=Location,
            topic="sensor_uwb_loaction",
            qos_profile=10,
        )
        self.uwb_location_pub

        self.header = [0xAC, 0xDA, 0x00, 0x03]
        self.temp_data = [0x00]*50
        self.state = 0
        self._cnt = 0
        self.valid_uwb_flag = False
        self.data_queue = []


    def __decoder(self, data):
        # 解算与A，B，C基站的距离
        DA = struct.unpack_from('>H', bytes(data[0:2]))[0]
        DB = struct.unpack_from('>H', bytes(data[2:4]))[0]
        DC = struct.unpack_from('>H', bytes(data[4:6]))[0]
        DD = struct.unpack_from('>H', bytes(data[6:8]))[0]

        # 解算X，Y，Z轴坐标
        X = struct.unpack_from('>h', bytes(data[-6:-4]))[0]
        Y = struct.unpack_from('>h', bytes(data[-4:-2]))[0]
        Z = struct.unpack_from('>h', bytes(data[-2:]))[0]
        return [DA, DB, DC, DD, X, Y, Z]

    def uwb_sampler_timer_callback(self):
        try:
            get_byte = self.ser.read(1)  # data byte

            if get_byte is not None and len(get_byte) > 0:

                u8_data = struct.unpack('>B', get_byte)[0]  # big endian

                # self.get_logger().info(
                #     f"u8_data: [{self._cnt}] [{u8_data:02x}]  {' '.join(['%02x'%d for d in self.temp_data[:self._cnt]])}")

                if self.state == 0 and u8_data == self.header[self.state]:
                    self.temp_data[self._cnt] = u8_data
                    self.state += 1
                    self._cnt += 1
                elif self.state == 1 and u8_data == self.header[self.state]:
                    self.temp_data[self._cnt] = u8_data
                    self.state += 1
                    self._cnt += 1
                elif self.state == 2 and u8_data == self.header[self.state]:
                    self.temp_data[self._cnt] = u8_data
                    self.state += 1
                    self._cnt += 1
                elif self.state == 3 and u8_data == self.header[self.state]:
                    self.temp_data[self._cnt] = u8_data
                    self.state += 1
                    self._cnt += 1
                elif self.state == 4:
                    self.temp_data[self._cnt] = u8_data
                    self._cnt += 1
                    if self._cnt >= len(self.header)+38:
                        
                        [DA, DB, DC, DD, X, Y, Z] = self.__decoder(
                            self.temp_data[len(self.header):self._cnt])
                        self.data_queue.append([X, Y, Z])
                        self.state = 0
                        self._cnt = 0
                        # self.get_logger().info(
                        #     f" A:{DA},B:{DB},C:{DC},D:{DD},  X:{X},Y:{Y},Z:{Z}"

                        # )
                        self.ser.flushInput()# 每次读取完数据后清空串口缓存区
                else:
                    self.state = 0
                    self._cnt = 0

        except Exception as e:
            pass

        if len(self.data_queue) > 10:
            xyz = [[], [], []]
            xyz_ave = [0, 0, 0]

            for i in range(len(self.data_queue)):
                xyz[0].append(self.data_queue[i][0])
                xyz[1].append(self.data_queue[i][1])
                xyz[2].append(self.data_queue[i][2])

            for idx in range(3):
                xyz[idx] = sorted(xyz[idx])
                xyz_ave[idx] = xyz[idx][int(len(xyz[idx]) / 2)]
                xyz_ave[idx] = sum(xyz[idx]) / len(xyz[idx])  # calc the mean
                xyz[idx].clear()

            # xyz_ave
            self.uwb_location_pub.publish(Location(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id="uwb",
                ),
                x=xyz_ave[0],
                y=xyz_ave[1],
                z=xyz_ave[2],
            ))
            # self.get_logger().info(f"uwb location: {xyz_ave}")

            self.data_queue.clear()


def main(args=None):
    rclpy.init(args=args)
    node = SensorUWB()
    rclpy.spin(node)
    # self.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
