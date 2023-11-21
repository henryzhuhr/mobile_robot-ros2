docker build -f docker/containers/Dockerfile.ros-desktop.foxy.jammy -t ros2:v1 .

docker run -it ros2:v1



RUN pip3 uninstall -y setuptools && pip3 install setuptools==58.2.0
