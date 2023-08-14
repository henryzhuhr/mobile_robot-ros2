# 手柄按键映射



ROS 内置手柄控制文档 http://wiki.ros.org/joy

Joy 消息结构
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

