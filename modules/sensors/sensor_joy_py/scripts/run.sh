script_dir=$(cd $(dirname $0); pwd)

source $script_dir/env/constant.sh

export WORKSPACE=$PWD
echo "work dir ${WORKSPACE}"
cd $WORKSPACE

# rm -rf build install 


# source /opt/ros/<dist>/setup.bash  in   ~/.bashrc
source ~/.bashrc
source .env/ros2/bin/activate


BUILD_LIST=(
    # # 接口
    state_interfaces # 系统状态接口

    # # 通用功能包
    # system_state # 系统状态
    
    # 系统控制
    # system_manager # 系统管理
    sensor_joy_py 


)
for item in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${item}${DEFAULT}"
    colcon build --packages-select ${item} \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug
    source install/setup.bash
done

source install/setup.bash

 ros2 launch sensor_joy_py run_joy.launch.py

