# 移动机器人控制 ROS2

基于 Jetson Nano

## 🍽️ 准备工作

### 摄像头畸变矫正

使用 [Opencv 官方棋盘格](https://docs.opencv.org/2.4/_downloads/pattern.png) 进行相机标定

<!-- 得出标定板的内部行列交点个数 `6 * 9` -->

<!-- ![棋盘格标定点](./images/camera-distortion-correction--checkerboard.png) -->

`camera-distortion-correction.py`


## 📦 功能包
### 视频流 cpp_video_streamer

目录: `src/cpp_video_streamer`

包含两个节点 
- 读取视频节点 `node_video_reader`
- 显示视频节点 `node_video_viewer`

#### 读取视频节点 node_video_reader
参数
- `source`: 视频源，可选择如下
  - `camera`: 默认，从摄像头读取
  - `<file/url>`:  文件名或者网络url
#### 显示视频节点 node_video_viewer



## 🚧 常见问题和解决方案


### Jetson Nano 上串口权限问题
```shell
could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0
```
把自己的用户加入到dialout组
```shell
sudo usermod -aG dialout ${USER}  # user 替换为自己的用户名
sudo usermod -aG dialout zx  # user 替换为自己的用户名
reboot							              # 必须要重启一下才会生效
```

### 串口名称问题

不同系统下串口名称可能不一样，需要根据实际情况修改，常见的串口名称如下：
- **Linux**: `/dev/ttyUSB*`, `/dev/ttyTHS*`(Jetson Nano 板载串口), `/dev/ttyACM*`(STM32下载/串口线)
- **Mac**: `/dev/tty.usbserial-*`, `/dev/tty.usbmodem-*`(STM32下载/串口线)
- **Jetson Nano** (参考 [User Guide](https://developer.nvidia.cn/embedded/learn/jetson-nano-2gb-devkit-user-guide))，有两组 UART: 
  - `/dev/ttyTHS1`: `UART_TX1` (8) / `UART_RX1` (10) ([40-Pin Header (J6)](https://developer.nvidia.cn/embedded/learn/jetson-nano-2gb-devkit-user-guide#id-.JetsonNano2GBDeveloperKitUserGuidevbatuu_v1.0-40-PinHeader(J6)))
  - `/dev/ttyTHS2`: `UART_RX2` (3) / `UART_TX2` (4) ([12-Pin Button Header (J12)](https://developer.nvidia.cn/embedded/learn/jetson-nano-2gb-devkit-user-guide#id-.JetsonNano2GBDeveloperKitUserGuidevbatuu_v1.0-12-PinButtonHeader(J12)))
  > 这里 UART 编号和网上的教程略有不同，这里参考的是官方文档




## 🔧 调试记录

### 车道线自动行进调试

- 2021.08.06:  
  1. 当前直走没有大问题
  2. 但是修正航向角时，转向存在转过头的问题，然后只能检测单线，导致无法正确前进
  3. 需要增加根据单车道线行进的逻辑。是否需要透视变换，把车道线拉垂直？