#!/usr/bin/env bash
#
# Start an instance of the jetson-inference docker container.
# See below or run this script with -h or --help to see usage options.
#
# This script should be run from the root dir of the jetson-inference project:
#
#     $ cd /path/to/your/jetson-inference
#     $ docker/run.sh
#



DOCKER_ROOT="/root/mobile_robot"	# where the project resides inside docker

# generate mount commands
DATA_VOLUME=" \
--volume $PWD/ros:$DOCKER_ROOT "

# parse user arguments

# USER_COMMAND="ros2 launch py_launch mobile_robot/package.yaml" 
USER_COMMAND="bash run.sh"
ROS_DISTRO="humble" #  , noetic, foxy, galactic, humble, iron

CONTINUE_RUN_WITHOUT_CONFIRM=false


source docker/color.sh

show_help() {
    echo " "
    echo "usage: Starts the Docker container and runs a user-specified command"
    echo " "
    echo "   ./docker/run.sh --container DOCKER_IMAGE"
    echo "                   --volume HOST_DIR:MOUNT_DIR"
    echo "                   --ros ROS_DISTRO"
    echo "                   --run RUN_COMMAND"
    echo " "
    echo "args:"
    echo " "
    echo "   --help                       Show this help text and quit"
    echo " "
    echo "   -c, --container DOCKER_IMAGE Specifies the name of the Docker container"
    echo "                                image to use (default: 'jetson-inference')"
    echo " "
    echo "   --dev  Runs the container in development mode, where the source"
    echo "          files are mounted into the container dynamically, so they"
    echo "          can more easily be edited from the host machine."
    echo " "
    echo "   -v, --volume HOST_DIR:MOUNT_DIR Mount a path from the host system into"
    echo "                                   the container.  Should be specified as:"
    echo " "
    echo "                                      -v /my/host/path:/my/container/path"
    echo " "
    echo "                                   These should be absolute paths, and you"
    echo "                                   can specify multiple --volume options."
    echo " "
    echo "  --ros DISTRO Specifies the ROS distro to use, one of:"
    echo "                 'noetic', 'foxy', 'galactic', 'humble', 'iron'"
    echo "               This will enable the use of ros_deep_learning package."
    echo "               When run with just --ros flag, the default distro is foxy."
    echo " "
    echo "   -r, --run RUN_COMMAND  Command to run once the container is started."
    echo "                          Note that this argument must be invoked last,"
    echo "                          as all further arguments will form the command."
    echo "                          If no run command is specified, an interactive"
    echo "                          terminal into the container will be provided."
    echo "   -y, Run without confirmation"
    echo " "
}


while :; do
    case $1 in
        -h|-\?|--help)
            show_help
            exit
            ;;
        -c|--container)  # takes an option argument; ensure it has been specified.
            if [ "$2" ]; then
                CONTAINER_IMAGE=$2
                shift
            else
                die 'ERROR: "--container" requires a non-empty option argument.'
            fi
            ;;
        --container=?*)
            CONTAINER_IMAGE=${1#*=} # delete everything up to "=" and assign the remainder.
            ;;
        --container=)  # handle the case of an empty flag
            die 'ERROR: "--container" requires a non-empty option argument.'
            ;;
        --dev)
            DEV_VOLUME=" --volume $PWD:$DOCKER_ROOT "
            ;;
        -v|--volume)
            if [ "$2" ]; then
                USER_VOLUME="$USER_VOLUME --volume $2 "
                shift
            else
                die 'ERROR: "--volume" requires a non-empty option argument.'
            fi
            ;;
        --volume=?*)
            USER_VOLUME="$USER_VOLUME --volume ${1#*=} "
            ;;
        --volume=)
            die 'ERROR: "--volume" requires a non-empty option argument.'
            ;;
        --ros)
            if [ "$2" ]; then
                ROS_DISTRO=$2
                shift
            else
                ROS_DISTRO="foxy"
            fi
            ;;
        --ros=?*)
            ROS_DISTRO=${1#*=}
            ;;
        --ros=)
            die 'ERROR: "--ros" requires a non-empty option argument.'
            ;;
        -r|--run)
            if [ "$2" ]; then
                shift
                USER_COMMAND=" $@ "
            else
                die 'ERROR: "--run" requires a non-empty option argument.'
            fi
            ;;
        -y)
            CONTINUE_RUN_WITHOUT_CONFIRM=true
            ;;
        --)
            shift
            break
            ;;
        -?*)
            printf 'WARN: Unknown option (ignored): %s\n' "$1" >&2
            ;;
        *)   # default case: No more options, so break out of the loop.
            break
    esac

    shift
done


# select container tag (unless specified by user) 
# `-z` is true if the length of string is zero
if [ -z "$CONTAINER_IMAGE" ]; then
	source docker/tag.sh
else
	source docker/containers/scripts/l4t_version.sh
fi


# check for V4L2 devices
V4L2_DEVICES=""
for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done


# check for display
DISPLAY_DEVICE=""
if [ -n "$DISPLAY" ]; then
	sudo xhost +si:localuser:root
	DISPLAY_DEVICE=" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix "
fi


echo "${LGREEN} info summary: ${DEFAULT}"
echo "$INFO${LGREEN}ros distro${DEFAULT}: ${LCYAN}$ROS_DISTRO${DEFAULT}"
echo "$INFO${LGREEN}container image${DEFAULT}: ${LCYAN}$CONTAINER_IMAGE${DEFAULT}"
echo "$INFO${LGREEN}V4L2 devices${DEFAULT}: ${LCYAN}$V4L2_DEVICES${DEFAULT}"
echo "$INFO${LGREEN}display${DEFAULT}: ${LCYAN}$DISPLAY_DEVICE${DEFAULT}"

echo "$INFO${LGREEN}data volume${DEFAULT}: ${LCYAN}$DATA_VOLUME${DEFAULT}"
echo "$INFO${LGREEN}user command${DEFAULT}: ${LCYAN}$USER_COMMAND${DEFAULT}"

#!/bin/bash
if [ "$CONTINUE_RUN_WITHOUT_CONFIRM" = false ] ; then
    while true; do
        read -p "Confirm and continue?(y/n) " answer
        case $answer in
            [yY]* ) echo "${LCYAN} Start run docker ${DEFAULT}"; break;;
            [nN]* ) exit;;
            * ) echo "Confirm and continue?(y/n)";;
        esac
    done
fi

USER_DEVICE=""
# run the container
if [ $ARCH = "aarch64" ]; then

	# /proc or /sys files aren't mountable into docker
	cat /proc/device-tree/model > /tmp/nv_jetson_model

    # 查找存在的 /dev/ttyUSB* 并挂载
    ls /dev/ttyUSB* > /tmp/ttyUSB
    if [ -s /tmp/ttyUSB ]; then
        while read line
        do
            echo "$INFO${LGREEN}Find Serial port${DEFAULT}: $line"
            USER_DEVICE="$USER_DEVICE --device=$line "
        done < /tmp/ttyUSB
        echo "$INFO${LGREEN}Add to docker command${DEFAULT}: ${USER_DEVICE}"
    else
        echo "$WARNING${LYELLOW}Serial Port [ttyUSB*] not exist${DEFAULT}"
        # exit 1
    fi

    # 挂载 /dev/ttyTH1 串口 40pin引脚上 8/10 
    /dev/ttyTH
    USER_DEVICE="$USER_DEVICE --device=/dev/ttyTH1 "


    # --runtime nvidia 使用nvidia-docker运行容器，nvidia 运行时会自动处理所有与NVIDIA GPU相关的设置，使容器能够使用GPU。
    # 请注意，你必须在Docker配置中预先定义运行时才能使用它。在Docker的配置文件（默认为/etc/docker/daemon.json）中，可以添加一个runtimes部分来定义运行时。例如：
    # {
    #     "runtimes": {
    #         "nvidia": {
    #             "path": "/usr/bin/nvidia-container-runtime",
    #             "runtimeArgs": []
    #         }
    #     }
    # }
    # -i 以交互模式运行容器
    # -t 为容器重新分配一个伪输入终端
    # -it 通常一起使用
    # --rm 容器退出时自动清理容器内部的文件系统
    # -v 挂载目录  `-v 容器外部文件夹路径:容器内部文件夹路径`
    # -w 指定容器的工作目录
	# sudo docker run --runtime nvidia -it --rm \
	sudo docker run --runtime nvidia -it --rm \
        --network host \
        --privileged=true \
		-v /tmp/argus_socket:/tmp/argus_socket \
		-v /etc/enctune.conf:/etc/enctune.conf \
		-v /etc/nv_tegra_release:/etc/nv_tegra_release \
		-v /tmp/nv_jetson_model:/tmp/nv_jetson_model  \
        -w $DOCKER_ROOT \
        $USER_DEVICE \
		$DISPLAY_DEVICE $V4L2_DEVICES \
		$DATA_VOLUME \
		$CONTAINER_IMAGE \
        $USER_COMMAND
else
    echo "$ERROR ${LRED}This script is only for Jetson${DEFAULT}"
fi