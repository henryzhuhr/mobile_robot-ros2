USER_COMMAND="bash core/controller/motion_controller_py/scripts/run-test.bash"
USER_COMMAND="/bin/bash"


CONTINUE_RUN_WITHOUT_CONFIRM=false
while :; do
    case $1 in
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

CONTAINER_IMAGE="docker/containers/Dockerfile.ros-humble-desktop.aarch64.l4t"
IMAGE_NAME="ros-humble-desktop"
TAG_NAME="aarch64-l4t"

DOCKER_ROOT="/root/mobile_robot"	# where the project resides inside docker

# generate mount commands
DATA_VOLUME="--volume $PWD:$DOCKER_ROOT"

source docker/color.sh

# check for V4L2 devices
V4L2_DEVICES=""
for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done
if [[ "$V4L2_DEVICES" = "" ]]; then
    V4L2_DEVICES_INFO="${LYELLOW}(warning: V4L2 devices not found)${DEFAULT}"
else
    V4L2_DEVICES_INFO="${LCYAN}$V4L2_DEVICES${DEFAULT}"
fi

# check for display
DISPLAY_DEVICE=""
if [ -n "$DISPLAY" ]; then
	sudo xhost +si:localuser:root
	DISPLAY_DEVICE=" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix "
fi
if [[ "$DISPLAY_DEVICE" = "" ]]; then
    DISPLAY_DEVICE_INFO="${LYELLOW}(warning: display not found)${DEFAULT}"
else
    DISPLAY_DEVICE_INFO="${LCYAN}$DISPLAY_DEVICE${DEFAULT}"
fi

USER_DEVICE=""
# run the container
if [ $ARCH = "aarch64" ]; then

	# /proc or /sys files aren't mountable into docker
	cat /proc/device-tree/model > /tmp/nv_jetson_model

    # Find existing /dev/ttyUSB* and mount
    ls /dev/ttyUSB* > /tmp/ttyUSB
    if [ -s /tmp/ttyUSB ]; then
        while read line
        do
            echo "$INFO${LGREEN}Find Serial port${DEFAULT}: $line"
            USER_DEVICE="$USER_DEVICE --device=$line"
        done < /tmp/ttyUSB
        echo "$INFO${LGREEN}Add to docker command${DEFAULT}: ${USER_DEVICE}"
    else
        echo "$WARNING${LYELLOW}Serial Port [ttyUSB*] not exist${DEFAULT}"
        # exit 1
    fi
    # Find existing /dev/ttyTHS* and mount
    ls /dev/ttyTHS* > /tmp/ttyTHS
    if [ -s /tmp/ttyTHS ]; then
        while read line
        do
            echo "$INFO${LGREEN}Find Serial port${DEFAULT}: $line"
            USER_DEVICE="$USER_DEVICE --device=$line"
        done < /tmp/ttyTHS
        echo "$INFO${LGREEN}Add to docker command${DEFAULT}: ${USER_DEVICE}"
    else
        echo "$WARNING${LYELLOW}Serial Port [ttyTHS*] not exist${DEFAULT}"
        # exit 1
    fi
fi


echo ""
echo ""
echo "${LGREEN} info summary: ${DEFAULT}"
echo ""
echo "$INFO${LGREEN}ros distro${DEFAULT}: ${LCYAN}$ROS_DISTRO${DEFAULT}"
echo "$INFO${LGREEN}container image${DEFAULT}: ${LCYAN}$CONTAINER_IMAGE${DEFAULT}"
echo "$INFO${LGREEN}V4L2 devices${DEFAULT}: ${LCYAN}$V4L2_DEVICES_INFO${DEFAULT}"
echo "$INFO${LGREEN}display${DEFAULT}: ${LCYAN}$DISPLAY_DEVICE_INFO${DEFAULT}"

echo "$INFO${LGREEN}data volume${DEFAULT}: ${LCYAN}$DATA_VOLUME${DEFAULT}"
echo "$INFO${LGREEN}user command${DEFAULT}: ${LCYAN}$USER_COMMAND${DEFAULT}"

echo "$INFO${LGREEN}user device${DEFAULT}: ${LCYAN}$USER_DEVICE${DEFAULT}"


#!/bin/bash
if [ "$CONTINUE_RUN_WITHOUT_CONFIRM" = false ] ; then
    while true; do
        read -p "Confirm and continue?(y/n) " answer
        case $answer in
            [yY]* ) echo "${LCYAN} Start run docker ${DEFAULT}"; break;;
            [nN]* ) echo "Terminate run docker, exit."; exit;;
            * ) echo "Confirm and continue?(y/n)";;
        esac
    done
fi


sudo docker build -t $IMAGE_NAME:$TAG_NAME -f $CONTAINER_IMAGE .

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
    $IMAGE_NAME:$TAG_NAME \
    $USER_COMMAND