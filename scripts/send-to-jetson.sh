IP_Jetson_nano="192.168.101.225"

parent_dir=$(cd "$(dirname "$PWD")"; pwd)
project_name=${PWD##*/}


SOURCE_DIR=$parent_dir/$project_name
SOURCE_DIR=$PWD/ros
TARGET_DIR="nano@$IP_Jetson_nano:~/project/mobile_robot-ros2"

echo " --> $SOURCE_DIR $TARGET_DIR"

# rsync -azv \
#     --exclude=".git/" --exclude=".gitignore/" --exclude=".github/" --exclude=".idea/" \
#     --exclude="package.json" --exclude="yarn.lock" --exclude="node_modules/" \
#     --exclude="__pycache__/" \
#     --exclude="docs/" --exclude="ros2-package" \
#     --exclude="ros/build/" --exclude="ros/install/" --exclude="ros/log/" \
#     $SOURCE_DIR $TARGET_DIR


# rsync -azv \
#     --exclude="__pycache__/" \
#     --exclude="ros/build/" --exclude="ros/install/" --exclude="ros/log/" --exclude="ros/.vscode/" \
#     $PWD/src $TARGET_DIR


rsync -azv $PWD/docker $TARGET_DIR
# rsync -azv $PWD/src $TARGET_DIR
# rsync -azv $PWD/public $TARGET_DIR
rsync -azv $PWD/scripts $TARGET_DIR

