source scripts/constant.sh

export WORKDIR=$PWD
echo "work dir ${WORKDIR}"
cd $WORKDIR

source ~/.bashrc
source .env/ros2/bin/activate
source /opt/ros/humble/setup.bash

source install/setup.bash

which python
ext_python_path=$(python -c 'import site; print(":".join(site.getsitepackages()))')
export PYTHONPATH=$PYTHONPATH:$ext_python_path


source install/setup.bash
ros2 run system_manager test_system_manager


