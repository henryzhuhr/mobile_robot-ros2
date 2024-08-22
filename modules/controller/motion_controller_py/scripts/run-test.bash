# ------ Parameter settings ------
PACKAGE_NAME="motion_controller_py"
LAUNCH_FILE="motion_controller.launch.py"     # 如果存在 launch 文件，优先启动 launch 文件
EXE_NODE="motion_controller_node"
REBUILD=false
# ------------------------------——


while [[ $# -gt 0 ]]; do
  case "$1" in
    -b | --rebuild )
      REBUILD=true;
      shift ;;
    -h | --help )
      echo "Autorun ROS"
      echo "If recompilation is required, add -b or --rebuild"
      exit 1
      ;;
    * )
      shift ;;
  esac
done

export WORKSPACE=$PWD

# 依赖包 和 当前包
BUILD_LIST=(
    $PACKAGE_NAME
)
# 预编译通用包，避免重复编译耗时，如果需要重新编译，添加 -b 参数或者手动删除 build 和 install 文件夹
PRE_BUILD_LIST=(    
    # - 接口 interfaces
    state_interfaces # 系统状态接口
    # - 通用功能包 common
    system_state_py # 系统状态 Python
    system_state    # 系统状态 C++
    base_node_py
    # - 系统管理器 manager
    system_manager # 系统管理
    # - 串口
    # serial
)


# -- add "source /opt/ros/<dist>/setup.bash"  in  ~/.bashrc
source ~/.bashrc
source /opt/ros/humble/setup.bash
source $WORKSPACE/scripts/env/constant.sh

echo "${LGREEN}work dir ${WORKSPACE}${DEFAULT}"

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


if [ "$REBUILD" = true ]; then
    echo "${LGREEN}[INFO]${DEFAULT} clean directory: ${LCYAN}build install${DEFAULT}"
    echo "${LGREEN}[INFO]${DEFAULT} rebuild package: ${LCYAN}${PRE_BUILD_LIST[@]}${DEFAULT}"
    rm -rf build install
fi

for pkg in ${PRE_BUILD_LIST[@]}; do
    if [ ! -d $WORKSPACE/install/$pkg ]; then
        echo ""
        echo "${LGREEN}Build: ${pkg}${DEFAULT}"
        colcon build --packages-select ${pkg} --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
        source install/setup.bash
    fi
done
source install/setup.bash


for pkg in ${BUILD_LIST[@]}; do
    echo ""
    echo "${LGREEN}Build: ${pkg}${DEFAULT}"
    colcon build --packages-select ${pkg} --symlink-install \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Debug \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=1
done
source install/setup.bash


if [ "$LAUNCH_FILE" != "" ]; then
    LAUNCH_FILE_PATH="install/$PACKAGE_NAME/share/$PACKAGE_NAME/$LAUNCH_FILE"   # 该路径基于 setup.py 中安装 launch 文件的路径
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
    if [ -f "install/$PACKAGE_NAME/lib/$PACKAGE_NAME/$EXE_NODE" ]; then
        ros2 run $PACKAGE_NAME $EXE_NODE
    else
        echo ""
        echo "${LRED}[ERROR] build \"$PACKAGE_NAME\" failed${DEFAULT} (from the script you run now)"
        echo ""
        echo "${LRED}        \"$PACKAGE_NAME:$EXE_NODE\" not found ${DEFAULT}"
        echo ""
    fi
fi