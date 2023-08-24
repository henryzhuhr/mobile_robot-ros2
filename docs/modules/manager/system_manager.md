# System Manager (系统状态管理器)

系统状态管理器，是系统的全局状态管理。


## 系统状态设计

系统状态管理器，是系统的全局状态管理。

系统状态是系统的全局状态，包括系统的运行状态、系统的配置状态、系统的错误状态等等。系统状态的设计，是为了方便系统的运行状态的监控、系统的配置状态的修改、系统的错误状态的记录等等。

## 系统状态

系统状态消息 [`SystemState`](sources/interfaces/state_interfaces/msg/SystemState.msg) ，定义了系统的状态信息，消息格式如下：

| 字段          | 类型            | 描述                                                   |
| ------------- | --------------- | ------------------------------------------------------ |
| header        | std_msgs/Header | 消息头                                                 |
| task_state    | uint64          | 位标识，系统当前执行的任务                             |
| task_enable   | uint64          | 位标识，系统任务列表可用性，0表示禁用                  |
| sensor_state  | uint64          | 位标识，系统当前使用的传感器，至多预留的64个传感器     |
| sensor_enable | uint64          | 位标识，系统传感器可用性，0表示禁用                    |
| vision_state  | uint64          | 位标识，系统当前执行的视觉算法，至多预留的64个视觉算法 |
| vision_enable | uint64          | 位标识，系统可用的视觉算法，0表示禁用                  |


> ROS2 消息参考 [common_interfaces](https://github.com/ros2/common_interfaces)

编程参考 [`test_system_manager`](sources/manager/system_manager/test/test_system_manager.cpp)