export DEFAULT=$(echo -en '\033[0m')
export LGREEN=$(echo -en '\033[01;32m')
export LCYAN=$(echo -en '\033[01;36m')
export LYELLOW=$(echo -en '\033[01;33m')
export LRED=$(echo -en '\033[01;31m')

export INFO="${LGREEN}[INFO]${DEFAULT}"
export WARNING="${LYELLOW}[WARNING]${DEFAULT}"
export ERROR="${LRED}[ERROR]${DEFAULT}"


export ENV_DIR=$PWD/.env/ros2


export PROJECT_NAME=mobile_robot-ros2