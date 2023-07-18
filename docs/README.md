# ROS2 Development


下载 [ROS2](https://github.com/ros2/ros2/releases) 

Documentation is at https://docs.ros.org


## 安装

### 安装 docker 


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



### 安装ROS2

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


## ROS 
[教程](http://www.autolabor.com.cn/book/ROSTutorials/di-2-zhang-ros-jia-gou-she-ji.html)