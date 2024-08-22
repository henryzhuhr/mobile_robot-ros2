export DEFAULT=$(echo -en '\033[0m')
export LGREEN=$(echo -en '\033[01;32m')
export LYELLOW=$(echo -en '\033[01;33m')
export LRED=$(echo -en '\033[01;31m')


TEST_PACKAGE="motion_controller_py" # 
TEST__NODE__="test_motion_controller"

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


source install/setup.bash

if [ -f "install/$TEST_PACKAGE/lib/$TEST_PACKAGE/$TEST__NODE__" ]; then
    ros2 run $TEST_PACKAGE $TEST__NODE__
else
    echo ""
    echo "${LRED}[ERROR] build \"$TEST_PACKAGE\" failed${DEFAULT} (from the script you run now)"
    echo "${LRED}        \"$TEST_PACKAGE:$TEST__NODE__\" not found ${DEFAULT}"
    echo ""
fi


