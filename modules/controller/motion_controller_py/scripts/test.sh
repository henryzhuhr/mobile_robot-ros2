export DEFAULT=$(echo -en '\033[0m')
export LGREEN=$(echo -en '\033[01;32m')
export LYELLOW=$(echo -en '\033[01;33m')
export LRED=$(echo -en '\033[01;31m')

# rm -rf build install log

TEST_PACKAGE="motion_controller_py" # 
TEST__NODE__="motion_controller"


BUILD_LIST=(
    # 数据接口
    state_interfaces # 系统状态接口

    # 通用功能模块
    system_state_py # 系统状态
    base_node_py # 基本节点

    sensor_joy_py # 传感器：手柄
    
    roslaunch

    $TEST_PACKAGE
)



export WORKSPACE=$PWD
echo "work dir ${WORKSPACE}"
cd $WORKSPACE



# source /opt/ros/<dist>/setup.bash  in   ~/.bashrc
source ~/.bashrc
if [ ! -d ".env/ros2/bin" ]; then
    echo ""
    echo "${LRED}[ERROR] ROS2 python env not found${DEFAULT}"
    echo ""
    echo "  create it by: ${LGREEN}\"python3 -m venv .env/ros2\"${DEFAULT}"
    echo ""
    exit 1
fi
source .env/ros2/bin/activate

ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path

for package in ${BUILD_LIST[@]}; do
    is_build=true
    if [ $package != "$TEST_PACKAGE" ]; then
        if [ -d "install/$package" ]; then
            is_build=false
        fi
    else
        if [ ! -d "build/$package" ]; then
            rm -rf build/$package
        fi
        if [ ! -d "install/$package" ]; then
            rm -rf install/$package
        fi
    fi
    
    if [ $is_build == true ]; then
        echo ""
        echo "${LGREEN}Build: ${package}${DEFAULT}"
        colcon build --packages-select ${package} \
            --symlink-install \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Debug
        source install/setup.bash
    fi
done



source install/setup.bash



if [ -f "install/$TEST_PACKAGE/lib/$TEST_PACKAGE/$TEST__NODE__" ]; then
    ros2 run $TEST_PACKAGE $TEST__NODE__
else
    echo ""
    echo "${LRED}[ERROR] build \"$TEST_PACKAGE\" failed${DEFAULT} (from the script you run now)"
    echo "${LRED}        \"$TEST_PACKAGE:$TEST__NODE__\" not found ${DEFAULT}"
    echo ""
fi


