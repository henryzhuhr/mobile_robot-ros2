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

[官方 ROS 镜像](https://hub.docker.com/_/ros/)，这里推荐使用 [OSRF Docker Images](https://github.com/osrf/docker_images)

