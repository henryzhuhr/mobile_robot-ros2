TMP_DIR=build_tmp
mkdir -p $TMP_DIR

# curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o $TMP_DIR/ros-archive-keyring.gpg

wget --quiet --show-progress --progress=bar:force:noscroll \
    --no-check-certificate https://nvidia.box.com/shared/static/5v89u6g5rb62fpz4lh0rz531ajo2t5ef.gz \
    -O $TMP_DIR/OpenCV-4.5.0-aarch64.tar.gz



# bash scripts/download-l4t-ros2.sh && bash scripts/up-to-jetson.sh