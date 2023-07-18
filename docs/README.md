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
```shell
# 进入文件夹并修改ros2-latest.list文件
cd /etc/apt/sources.list.d
sudo vim ros2-latest.list

# 打开文本后出现
deb http://packages.ros.org/ros2/ubuntu bionic main
# 在deb后插入[arch=amd64]
deb [arch=amd64] http://packages.ros.org/ros2/ubuntu bionic main

# 保存并关闭,然后更新
sudo apt update
# 最后就可以进行安装了
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


### docker 安装
通过 docker 安装，首先安装 docker
```shell
sudo apt update
sudo apt install -y docker.io
systemctl start docker  # 安装完成后启动docker
systemctl enable docker # 设置开机启动
docker version          # 查看docker版本
```

如果是非root用户，需要将用户加入docker用户组
```shell
sudo groupadd docker
sudo gpasswd -a ${USER} docker
```
通过配置 `/etc/docker/daemon.json​` 的方式，将`组`或者`用户`加入docker执行组
```json
{
    "group": "docker",
    "users": [
        "user1",
        "user2"
    ]
}
```
然后，执行 `sudo systemctl restart docker` ​重启守护进程。


[官方 ROS 镜像](https://hub.docker.com/_/ros/)，这里推荐使用 [OSRF Docker Images](https://github.com/osrf/docker_images)，已经写好了一个 `ros-desktop.foxy.jammy.dockerfile`
```shell
docker build -f dockerfiles/ros-desktop.foxy.jammy.dockerfile -t ros2:v1 .
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



### ROS2 服务
### ROS2 参数
### ROS2 动作