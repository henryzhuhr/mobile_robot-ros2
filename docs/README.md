# ROS2 Development


下载 [ROS2](https://github.com/ros2/ros2/releases) 

Documentation is at https://docs.ros.org


## 安装

推荐 apt 安装，docker 安装也可以，但是 docker 默认不支持图形界面

### apt 安装

1. 首先确保环境支持UTF-8
```shell
locale  # 检查系统是否安装有UTF-8
# 如果没有则进行安装
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # 安装好后再次检查
```

2. 设置源
```shell
# 添加apt仓库
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl http://repo.ros2.org/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
```

3. 安装package
设置软件源
```shell
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-foxy-desktop
```

4. 运行实例
```shell
# 打开一个终端并配置环境
source /opt/ros/foxy/setup.bash
# 运行C++ talker
ros2 run demo_nodes_cpp talker

# 然后打开另外一个终端并配置环境
source /opt/ros/foxy/setup.bash
# 运行C++ listener
ros2 run demo_nodes_py listener
```
需要提前安装 `rosdep` 和 `colcon`
```shell
sudo apt install -y python3-colcon-common-extensions
sudo apt install python3-rosdep2
sudo rosdep init # 需要🪜 
rosdep update
```

安装 `rosdep` 时，如果出现网络错误，`/etc/hosts` ，添加 ip 映射
```conf
185.199.110.133 raw.githubusercontent.com
```

网不好，编辑 `/etc/ros/rosdep/sources.list.d/20-default.list` 文件，添加如下内容
```shell
# os-specific listings first
yaml hhttps://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
yaml hhttps://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/base.yaml
yaml hhttps://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/python.yaml
yaml hhttps://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro hhttps://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
```

安装 python 依赖
```shell
pip3 uninstall empy -y
pip3 install empy lark numpy
```


### docker 安装
通过 docker 安装，首先安装 docker
```shell
sudo apt update
sudo apt install -y docker.io
systemctl start docker  # 安装完成后启动docker
systemctl enable docker # 设置开机启动
docker version          # 查看docker版本
```

如果是非 root 用户，需要将用户加入 docker 用户组
```shell
sudo groupadd docker
sudo gpasswd -a ${USER} docker
groups $USER
```
通过配置 `/etc/docker/daemon.json` 文件来配置 docker 镜像加速
```json
{
  "registry-mirrors": ["https://registry.docker-cn.com"]
}

```
然后，执行 `sudo systemctl restart docker` ​重启守护进程。


[官方 ROS 镜像](https://hub.docker.com/_/ros/)，这里推荐使用 [OSRF Docker Images](https://github.com/osrf/docker_images)，已经写好了一个 `Dockerfile.ros-desktop.foxy.jammy`
```shell
docker build -f docker/containers/Dockerfile.ros-desktop.foxy.jammy -t ros2:v1 .
docker run -it ros2:v1
```

运行ROS小海龟
```shell
ros2 run turtlesim turtlesim_node
```

但是由于 docker 默认不支持图形界面，所以需要配置一下
```shell
sudo apt-get install x11-xserver-utils
xhost + # 每次开机都要运行
#输出为：access control disabled, clients can connect from any host
```

重新启动一个容器，运行如下命令， `-d` 后台运行，`-it` 交互式运行，`-v` 挂载目录，`-e` 设置环境变量
```shell
docker run -d -it \
    -v /etc/localtime:/etc/localtime:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=unix$DISPLAY \
    -e GDK_SCALE \
    -e GDK_DPI_SCALE \
    ros2:v1
```

### 在 Jetson Nano 上安装 ROS2

Jetson Nano 安装 ROS2 只能通过 docker 安装，参考项目 [`dusty-nv/ros_deep_learning`](https://github.com/dusty-nv/ros_deep_learning)

将用户加入 docker 用户组
```shell
sudo gpasswd -a ${USER} docker
groups $USER
```
通过配置 `/etc/docker/daemon.json` 文件来配置 docker 镜像加速
```json
{
  "registry-mirrors": ["https://registry.docker-cn.com"]
}

```
然后，执行 `sudo systemctl restart docker` ​重启守护进程。

安装时参考 [ROS2 版本支持](http://dev.ros2.fishros.com/doc/Releases.html)，具体镜像在 [`dusty-nv/jetson-containers`](https://github.com/dusty-nv/jetson-containers)，修改 `docker/tag.sh` 中 `CONTAINER_IMAGE` 或者在运行时指定
```shell
bash docker/run-nano.sh # 默认 --ros humble, -y 跳过确认
```

进入容器后，运行
```shell
ros2 launch <package_name> <launch_file> <launch_arguments>
```


### VSCode 插件

安装 [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)，调出命令 `ctrl/cmd + shift + p `
- `ROS: Update C++ Properties`: 自动更新 `.vscode/c_cpp_properties.json` 中 C++ 配置，头文件目录等
- `ROS: Update Python Path`: 自动更新 `.vscode/settings.json` 中 Python 包路径


## ROS2 基础

[ROS2 中文网](http://dev.ros2.fishros.com)


### ROS2 节点

ROS 图是 ROS2 元素同时处理数据的网络。它包含所有可执行文件以及它们之间的连接 (如果要将它们全部映射并可视化)。

ROS 中的每个节点应负责单个模块目的 (例如，一个用于控制车轮电机的节点，一个用于控制激光测距仪的节点等)。每个节点可以通过话题、服务、动作或参数向其他节点发送和接收数据。 

一个完整的机器人系统由许多协同工作的节点组成。在 ROS2 中，单个可执行文件 (cprogram程序、Python程序等) 可以包含一个或多个节点。

![ROS2](./images/Nodes-TopicandService.gif)


ROS2 命令
```shell
ros2 run <package_name> <executable_name>
ros2 run turtlesim turtlesim_node # 启动小海龟
```

然而，我们仍然不知道节点名称。您可以使用 `ros2 node list` 找到节点名称

当 turtlesim 仍在另一个终端中运行时，打开一个新终端，然后输入以下命令，可以看到输出 `/turtlesim` ，这是 turtlesim 节点的名称
```shell
ros2 node list
```
再打开一个终端启动另一个节点 `ros2 run turtlesim turtle_teleop_key` ，然后再次运行 `ros2 node list` ，可以看到输出 `turtlesim_node` 和 `/teleop_turtle`



访问结点信息，可以看到节点的名称、类型、发布的话题、订阅的话题、提供的服务、使用的服务、使用的参数
```shell
ros2 node info /turtlesim
```


### ROS2 话题

ROS2 将复杂的系统分解成许多模块化节点，**话题 (Topic)** 是 ROS2 中最常用的通信机制，它充当节点交换消息的总线，是一种**发布/订阅机制**。话题是数据在节点之间移动，从而在系统的不同部分之间移动的主要方式之一。

节点可以将数据发布到任意数量的话题，同时订阅任意数量的话题。

![Topic-SPSS](./images/Topic-SinglePublisherandSingleSubscriber.gif)
![Topic-MPMS](./images/Topic-MultiplePublisherandMultipleSubscriber.gif)


在新终端中运行 `ros2 topic list` 命令将返回系统中当前活动的所有话题的列表:
```shell
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

`ros2 topic list -t` 返回相同的话题列表，这次在括号中附加了话题类型
```shell
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```


### ROS2 服务
服务是 ROS 图中节点的另一种通讯方法。服务基于调用和响应模型，而不是话题的发布者-订阅者模型。而话题允许节点订阅数据流得到不断更新，服务对具体调用的客户端提供数据。 

还是现在两个终端分别运行 `ros2 run turtlesim turtlesim_node` `ros2 run turtlesim turtle_teleop_key`
#### 服务列表

在新终端中运行 ros2 service list 命令将返回系统中当前活动的所有服务的列表: [待校准@8188]
```shell
/clear
/kill
/reset
/spawn
/teleop_turtle/describe_parameters
...
```
> 你将会看到两个节点都有相同的六个服务，它们的名字中有 `parameters` 。几乎 ROS 2 中的每个节点都有这些构建参数的基础设施服务。

可以看到 turtlesim特定服务， `/clear`, `/kill`, `/reset`, ` spawn`, `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/turtle1/teleport_relative`


### ROS2 参数
### ROS2 动作

## 创建 ROS 包
包可以被视为 ROS2 代码的容器。如果你想安装你的代码或者与其他人共享，那么你需要把它组织成一个包。

使用 CMake 或 Python 创建一个新包，并运行其可执行文件。

ROS2 中的包创建使用 ament 作为其构建系统，colcon 作为其构建工具。您可以使用官方支持的 CMake 或 Python 创建包，尽管确实存在其他构建类型。


工作区中包的结构如下：
```shell
workspace_folder # 这里指目录下 ros
├── src
│   ├── package_1 # 基于 CMake 的包
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── package_2 # 基于 Python 的包
│   │   ├── setup.py
│   │   ├── package.xml
│   │   ├── package_2
│   │   │   ├── __init__.py
│   │   │   ├── publisher_member_function.py
│   │   │   └── ...
│   │   └── resource/package_2
│   └── ...
└── ...
```

接下来需要创建包，构建类型有 `cmake`, `ament_cmake`, `ament_python`
```shell
cd src
ros2 pkg create --build-type ament_python  <pack_name>
```



### 编写发布和订阅包 (Python)

创建名为 `py_pubsub` 的包
```shell
cd src
ros2 pkg create --build-type ament_python py_pubsub
```
得到如下内容
```shell
package_2 # 基于 Python 的包
├── setup.py
├── package.xml
└── package_2
    ├── __init__.py
    ├── publisher_member_function.py    # 创建该文件，发布者
    ├── subscriber_member_function.py   # 创建该文件，订阅者
    └── ...
```
<!-- http://dev.ros2.fishros.com/doc/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html -->

在构建之前，在工作区的**根目录**下运行 rosdep ，以检查是否缺少依赖项:
```shell
rosdep install -i --from-path src --rosdistro foxy -y
```

仍然在你的工作空间的根，建立你的新的包:
```shell
colcon build --packages-select py_pubsub
```

两个终端分别激活环境并且运行
```shell
. install/setup.bash
ros2 run py_pubsub talker
```
```shell
. install/setup.bash
ros2 run py_pubsub listener
```


### 编写发布和订阅包 (C++)

创建名为 `cpp_pubsub` 的包，使用 `ament_cmake` 构建类型
```shell
cd src
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

在包 `cpp_pubsub` 目录下添加 `src/publisher_member_function.cpp` 和 `src/subscriber_member_function.cpp` 文件，以及 `CMakeLists.txt` 文件，添加包依赖
```cmake
find_package(rclcpp REQUIRED)   # 添加依赖
find_package(std_msgs REQUIRED) # 添加依赖
```
在 `ament_package()` 之前添加编译节点可执行文件
```cmake
# [Node] talker : 编译 链接ROS 安装
add_executable(talker # 可执行文件名，也就是节点(运行的文件)名
  src/publisher_member_function.cpp # 源文件
)
# target_link_libraries(video_reader ${OpenCV_LIBRARIES}) # 如果有
ament_target_dependencies(talker
  rclcpp    # 添加 ROS 包: rclcpp
  std_msgs  # 添加 ROS 包: std_msgs
)
install(TARGETS     talker              # 安装的可执行文件
        DESTINATION lib/${PROJECT_NAME} # 安装的位置
)

# [Node] listener : 编译 链接ROS 安装
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})
```

编译
```shell
colcon build --packages-select cpp_pubsub
```

两个终端分别激活环境并且运行
```shell
. install/setup.bash
ros2 run cpp_pubsub talker
```
```shell
. install/setup.bash
ros2 run cpp_pubsub listener
```


### 编写服务和客户端包 (Python)

创建名为 `py_srvcli` 的包，并且添加依赖 `rclpy` 和 `example_interfaces`
```shell
cd src
ros2 pkg create py_srvcli \
    --build-type ament_python \
    --dependencies rclpy example_interfaces
```
也可以在创建时，添加包的相关信息
```shell
cd src
ros2 pkg create py_srvcli \
    --build-type ament_python \
    --dependencies rclpy example_interfaces \
    --description py_srvcli \
    --maintainer-email "example_email@gmail.com" \
    --license "Apache License 2.0"
```

在 `src/py_srvcli/py_srvcli` 目录下创建服务端文件 [`service_member_function.py`](../src/py_srvcli/py_srvcli/service_member_function.py) 和客户端文件 `client_member_function.py` 

然后在 `src/py_srvcli/setup.py` 中添加入口起点
```python
entry_points={
    'console_scripts':[
      'service = py_srvcli.service_member_function:main',
      'client = py_srvcli.client_member_function:main',
    ],
},
```

在根目录下构建 `py_srvcli` 包
```shell
colcon build --packages-select py_srvcli
```

两个终端分别激活环境并且运行
```shell
. install/setup.bash
ros2 run py_srvcli service
```
```shell
. install/setup.bash
ros2 run py_srvcli client 2 3
```

### 编写服务和客户端包 (C++)

使用 C++ 创建和运行**服务**和**客户端**节点。当 `nodes` 使用 `services` 进行通信时，**发送数据请求的节点**被调用到**客户端节点**，**响应请求的节点**是**服务节**点。请求和响应的结构由 `.srv` 文件决定。

创建新包 `custom_interfaces`，使用 `ament_cmake` 构建类型
```shell
cd src
ros2 pkg create --build-type ament_cmake cpp_srvcli \
    --dependencies rclcpp custom_interfaces
# 依赖于刚才使用过的 custom_interfaces 包    
```

`custom_interfaces` 包内包含 `AddTwoInts.srv` 文件，前两行是请求的参数，破折号下面是响应。
```srv
int64 a
int64 b
---
int64 sum
```

在 `cpp_srvcli` 包内，创建 `src/add_two_ints_server.cpp`
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<custom_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<custom_interfaces::srv::AddTwoInts::Response> response)
{
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld b: %ld", request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // 初始化ROS2 c client客户端库

    // 创建一个名为 add_two_ints_server 的节点
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

    // 为该节点创建名为 add_two_ints 的服务，并自动调用y使用 “& add” 方法在网络上发布该服务
    rclcpp::Service<custom_interfaces::srv::AddTwoInts>::SharedPtr service =
        node->create_service<custom_interfaces::srv::AddTwoInts>("add_two_ints", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

    rclcpp::spin(node); // 旋转节点，使服务可用
    rclcpp::shutdown();
}
```


编辑客户端节源码 `add_two_ints_client.cpp`
```cpp
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }
    
    // 与服务节点类似，以下代码行创建节点，然后为该节点创建客户端:
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
    rclcpp::Client<custom_interfaces::srv::AddTwoInts>::SharedPtr client =
        node->create_client<custom_interfaces::srv::AddTwoInts>("add_two_ints");

    // 接下来，创建请求。它的结构是由前面提到的 .srv 文件定义的。
    auto request = std::make_shared<custom_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    // while 循环给客户端1秒钟搜索网络中的服务节点。如果找不到，它将继续等待。
    while (!client->wait_for_service(1s))
    {
        // 如果客户端被取消 (例如，通过您在终端中输入 “ctrl + c”)，它将返回一条错误日志消息，说明它被中断了。
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // 然后，客户端发送其请求，节点旋转直到收到响应或失败。

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");

    rclcpp::shutdown();
    return 0;
}
```

编辑 `CMakeLists.txt` ，
```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_srvcli)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# [Node] server : 编译、链接、安装
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp custom_interfaces)
install(TARGETS server DESTINATION lib/${PROJECT_NAME})

# [Node] client : 编译、链接、安装
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp custom_interfaces)
install(TARGETS client DESTINATION lib/${PROJECT_NAME})

ament_package()
```

构建
```shell
colcon build --packages-select cpp_srvcli
```

运行服务节点
```shell
source install/setup.bash
ros2 run cpp_srvcli server
```
另一个终端，运行客户端节点，给两个数作为参数
```shell
source install/setup.bash
ros2 run cpp_srvcli client 2 3
```



### 自定义接口 msg srv

自定义接口文件 ( `.msg` 和 `.srv` )，并将其与 Python 和 C++ 节点一起使用

创建新包 `custom_interfaces`，使用 `ament_cmake` 构建类型
```shell
cd src
ros2 pkg create --build-type ament_cmake custom_interfaces
```


在 `custom_interfaces` 目录 (和包同级目录) 下创建 `msg` 和 `srv` 文件夹，创建 `msg/Num.msg` 和 `srv/AddTwoInts.srv`
```shell
cd custom_interfaces
mkdir -p msg && mkdir -p srv
touch msg/Num.msg && touch srv/AddTwoInts.srv
```

在 `Num.msg` 中声明其数据结构。这是自定义消息，用于传输 `num`
```shell
int64 num
```

在 `AddTwoInts.srv` 中声明其数据结构。这是自定义服务，用于传输 `a` 和 `b`
```shell    
int64 a
int64 b
---
int64 sum
```

在 `CMakeLists.txt` 中添加如下内容
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "msg/Num.msg"
)
```

因为接口依赖于 `rosidl_default_generators` 来生成特定于语言的代码，所以您需要声明对它的依赖，在 `package.xml` 中添加如下内容。
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

构建 `custom_interfaces` 包
```shell
colcon build --packages-select custom_interfaces
```
然后，现在这些接口将被其他 ROS2 包发现并使用

通过使用 `ros2 interface show `命令确认您的接口创建有效
```shell
. install/setup.bash
ros2 interface show custom_interfaces/msg/Num
ros2 interface show custom_interfaces/srv/AddTwoInts
```

修改 `py_pubsub` 包，添加依赖 `custom_interfaces`，并且修改 `publisher_member_function.py` 和 `subscriber_member_function.py` 文件，以及 `setup.py` 文件

### 自定义参数 (Python)

使用 Python (rclpy) 创建并运行具有ROS参数的类。当你制作自己的 nodes 时，你有时需要添加可以从launch文件中设置的参数。

### 自定义参数 (C++)
创建并运行具有 ROS 参数的类。有时节点在运行前需要从 launch  文件中设置的参数。 

创建包 `cpp_parameters`
```shell
cd src
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
```

在包目录 `src/cpp_parameters` 下创建 `src/cpp_parameters_node.cpp`
```cpp
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class ParametersClass : public rclcpp::Node
{
private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    ParametersClass() : Node("parameter_node")
    {
        // 创建我们的参数。我们的参数名为 my_parameter ，并被指定为默认值 world
        this->declare_parameter<std::string>("my_parameter", "world");
        // timer_ 初始化，这导致 respond 函数每秒执行一次
        timer_ = this->create_wall_timer(1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
        // respond 函数的第一行从节点获取参数 my_parameter ，并将其存储在 parameter_string_ 中
        this->get_parameter("my_parameter", parameter_string_);
        RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParametersClass>()); // “rclcpp:: spin” 开始处理来自节点的数据。
    rclcpp::shutdown();
    return 0;
}
```

`CMakeLists.txt` 文件
```cmake
add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

install(TARGETS
  parameter_node
  DESTINATION lib/${PROJECT_NAME}
)
```

构建
```shell
colcon build --packages-select cpp_parameters
```

运行节点
```shell
source install/setup.bash
ros2 run cpp_parameters parameter_node
```
另一个终端，运行节点
```shell
source install/setup.bash
# 查看参数
ros2 param list
# 修改参数
ros2 param set /parameter_node my_parameter earth
```

可以通过 launch 文件修改，在包目录 `cpp_parameters` 下创建 `launch/cpp_parameters_launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
```

 `CMakeLists.txt` 文件添加
```cmake
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
```
运行节点
```shell
source install/setup.bash
ros2 launch cpp_parameters cpp_parameters_launch.py
```


## 启动文件 launch
ROS2 中提供了 launch 模块用于实现节点的批量启动

参考：https://www.wolai.com/kachex/9Cd3RkqmuNMRxu9HVB2YRz

创建一个 `py_launch` 的包用于批量启动节点。并在新建的包 `py_launch` 的目录下，创建 `launch` 目录
在 ros
```shell
ros2 pkg create py_launch --build-type ament_python --dependencies rclpy
mkdir -p py_launch/launch
```

在 `py_launch/launch` 目录下创建 launch 文件

**python 文件**：`py_all_nodes.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
    video_reader = Node(package="py_video", executable="video_reader", name="t1")
    return LaunchDescription([video_reader,])#  可以添加多个节点
```

**xml 文件：**`py_all_nodes.launch.xml`
```xml
<launch>
    <node pkg="turtlesim" exec="turtlesim_node" name="t1"/>
    <node pkg="turtlesim" exec="turtlesim_node" name="t2"/>
</launch>
```

**yaml 文件：**`py_all_nodes.launch.yaml`
```yaml
launch:
- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
    name: "t1"
- node:
    pkg: "turtlesim"
    exec: "turtlesim_node"
    name: "t2"
```


构建、运行
```shell
colcon build
. install/setup.bash
ros2 launch py_launch py_all_nodes.launch.py
```




## ROS2 小车

### 功能包
#### 视频流 cpp_video_streamer

包含两个节点 
- 读取视频节点 `node_video_reader`
- 显示视频节点 `node_video_viewer`

##### 读取视频节点 node_video_reader
参数
- `source`: 视频源，可选择如下
  - `camera`: 默认，从摄像头读取
  - `<file/url>`:  文件名或者网络url
##### 显示视频节点 node_video_viewer
