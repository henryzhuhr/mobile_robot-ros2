# 移动机器人开发文档

## 项目简介

涉及技术栈: 
- 语言层: Python3, C++
- 框架: ROS2, PyTorch,  TensorRT
- 库: OpenCV, TensorRT, MQTT, RTMP
- 软件包管理: CMake
- 软件开发相关: Git, Github Actions, VSCode, Docker

## 项目结构

![系统架构](./images/system-architecture.jpg)

该项目的 ROS2 工作空间包括 `core` 和 `modules` 

`core` 中包含功能包如下:

- **controller**: 系统控制模块。包括底盘控制、机械臂控制等
  - 🔄 [motion_controller](./core/controller/motion_controller.md): 运动控制模块
  - 💠 arm_controller: 机械臂控制模块

- **manager**: 系统管理模块。包括系统状态管理、任务管理等
  - **launch**: ROS2 启动文件
  - ✅ [system_manager](./core/manager/system_manager.md): 系统状态管理模块
  - 🔄 [task_manager](./core/manager/system_manager.md): 任务管理模块
  
- **base**: 基础模块。包括系统中的基础功能，如基本节点、基本传感器、基本深度学习模型等，便于统一开发
  - ✅ [base_node](./core/base/base_node.md): 为系统中功能节点的开发提供基础，包括节点的初始化、参数的读取、心跳包的发送等。 提供一个参考模板 [node_template](./modules/common/base_node.md)


`modules` 中包含功能包如下:
- **common**: 通用功能模块。包括全局的系统状态码定义、通用的工具函数等
  - ✅ system_state: 系统状态码、错误码的全局定义，模块的注册序号，话题、服务、参数的全局定义
  - 💠 utils: 通用工具函数

- **interfaces**: 数据接口。 包括各个模块之间的数据接口、消息结构定义
  - ✅ state_interfaces: 系统状态接口和服务接口

- **data_transmission**: 数据传输模块
  - 🔄 [dt_mqtt](./modules/data_transmission/dt_mqtt.md): MQTT 通信模块
  - 🚧 [dt_rtmp](./modules/data_transmission/dt_rtmp.md): 视频推流模块，RTMP 协议


- **sensors**: 传感器模块。包括各种传感器的驱动、数据处理等
  - ✅ [sensor_joy](./modules/sensors/sensor_joy.md): 手柄
  - 🔄 sensor_uwb: UWB 定位

- **vision**: 视觉算法模块。包括各种视觉算法的实现
  - ✅ vision_lanedet: 车道线检测算法
  - 🚧 vision_objdet: 目标检测算法
  - 💠 vision_objtrack: 目标检测算法

- **navigation**: 导航模块 (未开发)

- **app**: 应用程序
  - 💠 app_web: web 端控制面板 (Vue3 + flask)


功能开发流程: 
```shell
💠 计划开发 → 🚧 开发中 → 🔄 测试中 → ✅ 开发完成 → ⛔️ 下线
                        ⬆︎        ↙︎ 
                        🐛 修复中
                        🚀 升级中         
```
             


## 分支管理

项目使用 git 进行版本控制，每个功能模块都应该新建一个分支进行开发，开发完成后合并到 `dev` 分支，`dev` 分支的代码经过测试后 PR 到 `main` 分支。**功能模块命名规则**:

其他子模块的开发和测试是完全可以独立的，因此，其他功能包的分支应该为 `pkg-<group>__<package_name>` 结构，`<group>` 为功能分组，`<package_name>` 为功能包名，中间用双下划线 `__` 分隔。

例如，传感器 (sensors) 中摄像头 (camera) 的分支应该为 `pkg-sensors__camera` 


## 项目文档

拥有开发权限的开发者可以查看完整文档: 
```shell
yarn
yarn docs:dev
```

## 项目规范

- C++ 代码规范: [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) | [C++ 风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents/)
- Python 代码规范: [Google Python Style Guide](https://google.github.io/styleguide/pyguide.html) | [Python 风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-python-styleguide/contents/)
- Shell 代码规范: [Shell Style Guide](https://google.github.io/styleguide/shellguide.html) | [Shell 风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-shell-styleguide/contents/)

## 开发贡献
<br>
<a href="https://github.com/HenryZhuHR/mobile_robot-ros2/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=HenryZhuHR/mobile_robot-ros2" />
</a>
