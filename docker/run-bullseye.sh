# sudo docker build -f docker/containers/Dockerfile.ros-desktop.humble.bullseye -t ros2:v1 .

# sudo docker run -it ros2:v1

# RUN pip3 uninstall -y setuptools && pip3 install setuptools==58.2.0


# docker compose

# docker-compose -f docker/compose/docker-humble.bullseye.yml up -d 
# sudo docker-compose -f docker/compose/docker-humble.bullseye.yml down

sudo docker-compose -f docker/compose/docker-humble.bullseye.yml run ros bash