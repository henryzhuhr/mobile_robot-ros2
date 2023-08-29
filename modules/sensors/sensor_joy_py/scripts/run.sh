export WORKSPACE=$PWD
echo "${LGREEN}work dir ${WORKSPACE}${DEFAULT}"

PACKAGE_NAME="sensor_joy_py"
LAUNCH_FILE="run_joy.launch.py"     # 如果存在 launch 文件，优先启动 launch 文件
EXE_NODE=""


# 依赖包 和 当前包
BUILD_LIST=(
    $PACKAGE_NAME 
)

# rm -rf build install 
# 预编译通用包，避免重复编译耗时，如果需要重新编译，取消上一行注释
PRE_BUILD_LIST=(    
    # - 接口 interfaces
    state_interfaces # 系统状态接口
    # - 通用功能包 common
    system_state_py # 系统状态
    # - 系统管理器 manager
    system_manager # 系统管理
)


# -- add "source /opt/ros/<dist>/setup.bash"  in  ~/.bashrc
source ~/.bashrc
source $WORKSPACE/scripts/env/constant.sh
if [ -d ".env/ros2/bin" ]; then
    source .env/ros2/bin/activate
    ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
    export PYTHONPATH=$PYTHONPATH:$ext_python_path
else 
    echo ""
    echo "${LRED}[ERROR] ROS2 python env not found${DEFAULT}"
    echo ""
    echo "  create it by: ${LGREEN}\"python3 -m venv .env/ros2\"${DEFAULT}"
    echo ""
    exit 1
fi



for pkg in ${PRE_BUILD_LIST[@]}; do
    if [ ! -d $WORKSPACE/install/$pkg ]; then
        echo ""
        echo "${LGREEN}Build: ${pkg}${DEFAULT}"
        colcon build --packages-select ${pkg} --symlink-install
        source install/setup.bash
    fi
done
for pkg in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${pkg}${DEFAULT}"
    colcon build --packages-select ${pkg} --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug
    source install/setup.bash
done
source install/setup.bash


if [ "$LAUNCH_FILE" != "" ]; then
    LAUNCH_FILE_PATH="install/$PACKAGE_NAME/share/$PACKAGE_NAME/$LAUNCH_FILE"   # 该路径参考了 setup.py 中安装 launch 文件的路径
    if [ -f $LAUNCH_FILE_PATH ]; then
        ros2 launch $PACKAGE_NAME $LAUNCH_FILE
    else
        echo ""
        echo "${LRED}[ERROR] launch file \"$LAUNCH_FILE\" not found in:${DEFAULT}"
        echo ""
        echo "${LRED}       \"$LAUNCH_FILE_PATH\"${DEFAULT}"   
        echo ""
    fi
else
    if [ -f "install/$TEST_PACKAGE/lib/$TEST_PACKAGE/$TEST__NODE__" ]; then
        ros2 run $PACKAGE_NAME $EXE_NODE
    else
        echo ""
        echo "${LRED}[ERROR] build \"$TEST_PACKAGE\" failed${DEFAULT} (from the script you run now)"
        echo ""
        echo "${LRED}        \"$TEST_PACKAGE:$TEST__NODE__\" not found ${DEFAULT}"
        echo ""
    fi
fi



