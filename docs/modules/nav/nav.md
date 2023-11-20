根据 [Getting Started](https://navigation.ros.org/getting_started/index.html)安装，Nav2 和 [Turtlebot3](https://github.com/robotis-ros2-release/turtlebot3-release)
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

```bash
sudo apt install ros-humble-turtlebot3-gazebo
```


写入环境变量 

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```