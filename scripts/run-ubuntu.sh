source scripts/constant.sh

export WORKDIR=$PWD
echo "work dir ${WORKDIR}"
cd $WORKDIR




# -- 构建运行 --
cd $WORKDIR
# rm -rf build install log

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:"$WORKDIR/libs/jsoncpp/"
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:"$WORKDIR/libs/paho-mqtt/lib/cmake/eclipse-paho-mqtt-c"
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:"$WORKDIR/libs/paho-mqtt/lib/cmake/PahoMqttCpp"


source ~/.bashrc
source .env/ros2/bin/activate
source /opt/ros/humble/setup.bash

# eval "$(conda shell.bash hook)"
# conda activate ros2


# pip install empy numpy==1.20


BUILD_LIST=(
    # state_interfaces # 系统状态接口
    # system_manager # 系统管理
    dt_mqtt_py # 数据传输
    dt_rtmp_py # 视频流数据
    
    # video_streamer_cpp # 视频流
    # sensor_uwb_py     # uwb 数据采集功能
    # vision_lanedet_py     # 视觉 车道线检测
    # controller_py
    # py_launch
)
for item in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${item}${DEFAULT}"
    colcon build --packages-select ${item} \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug
    source install/setup.bash
done

which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path


# ros2 launch py_launch car.launch.py
# ros2 run system_manager system_manager
ros2 run dt_mqtt_py dt_mqtt




# ros2 run sensor_uwb_py sensor_uwb

