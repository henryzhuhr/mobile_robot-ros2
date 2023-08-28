# 手柄


## 概述


手柄将作为一个传感器 ，发布手柄摇杆量和按键状态消息。

其「状态分组」为 `SystemState::StateGroup::SENSOR`

其「组内注册序号」为 `SystemState::Sensor::SENSOR_JOY`

## 功能设计

检测手柄摇杆量和按键状态，发布消息。
- 将手柄摇杆量转化为控制速度发出，话题名称 `SystemState.topics.joy_speed` ，消息类型 `state_interfaces::msg::Speed` ，当检测到手柄摇杆量变化时发布。
- (TODO) 话题名称 `SystemState::topics::joy_button` ，消息类型 `state_interfaces::msg::Button` ，当检测到手柄按键变化时发布。


## 配置文件


ROS 内置了 [Joy 节点](http://wiki.ros.org/joy)，实时将 `/dev/input/js0` 的数据转化为 `sensor_msgs::Joy` 消息，其数据结构如下：
```python
sensor_msgs.msg.Joy(
    header=std_msgs.msg.Header(
        stamp=builtin_interfaces.msg.Time(
            sec=1690353442,
            nanosec=770217921
        ),
        frame_id='joy'
    ),
    axes=[
        -0.0,   # 左摇杆 x 轴   : 左+ 右-
        -0.0,   # 左摇杆 y 轴   : 上+ 下-
        -0.0,   # 右摇杆 y 轴   : 上+ 下-
        -0.0,   # 右摇杆 x 轴   : 左+ 右-
        0.0,    # 左侧方向键 x 轴: 左+ 右-
        0.0     # 左侧方向键 y 轴: 上+ 下-
        ],
    buttons=[
        0,      #  0 右侧按键 1
        0,      #  1 右侧按键 2
        0,      #  2 右侧按键 3
        0,      #  3 右侧按键 4
        0,      #  4 左上侧按键 1
        0,      #  5 右上侧按键 1
        0,      #  6 左上侧按键 2
        0,      #  7 右上侧按键 2
        0,      #  8 select
        0,      #  9 start
        0,      # 10 左摇杆按键
        0       # 11 右摇杆按键
        ]
)
```


但是不同手柄读取的摇杆量和按键状态不同，需要对其进行映射，并统一转化为控制速度和按键消息

映射文件(`.json`)中需要给出的键值，值为下标(`int`)：

axes:
- `axis_stick_left__LR`: 左摇杆，左右，默认左+右-
- `axis_stick_left__UD`: 左摇杆，上下，默认上+下-
- `axis_stick_right_LR`: 右摇杆，左右，默认左+右-
- `axis_stick_right_UD`: 右摇杆，上下，默认上+下-
- `axis_cross_LR`: 方向键，左右，默认左+右-
- `axis_cross_UD`: 方向键，上下，默认上+下-
- `axis_LT`: 左上侧按键 2，默认按下为 -1.0，松开为 1.0
- `axis_RT`: 右上侧按键 2，默认按下为 -1.0，松开为 1.0

buttons:
- `A`: 右侧按键 1
- `B`
- `X`
- `Y`
- `LB`: 左上侧按键 1
- `RB`: 右上侧按键 1
- `select`
- `start`
- `button_stick__left`: 左摇杆按键
- `button_stick_right`: 右摇杆按键


现有的映射文件：
- Microsoft Xbox 360 手柄 : `configs/sensors/sensor_joy/key_maps/Microsoft-X-Box-360-pad.json`
- 不知名的有线手柄 : `configs/sensors/sensor_joy/key_maps/wire_joy.json`



## 调试命令

- 在 `modules/sensors/sensor_joy_py/scripts/joy_test.py` 中，可以测试手柄的摇杆量和按键状态，而不依赖于 ROS 节点，目的是为了测试手柄功能是否正常
- `modules/sensors/sensor_joy_py/scripts/run.sh` 启动一个 launch 文件，同时启动 ROS 内置 Joy 节点和传感器节点，可以测试手柄功能是否正常，以及传感器节点是否正常发布消息