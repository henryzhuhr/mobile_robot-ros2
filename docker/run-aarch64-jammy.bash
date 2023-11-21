export DOCKER_COMPOSE_FILE="docker/compose/docker-compose.ros-humble-desktop.aarch64.jammy.yml"

# fast run
sudo docker compose -f ${DOCKER_COMPOSE_FILE} run ros bash 

# rm
# sudo docker-compose rm -f ${DOCKER_COMPOSE_FILE} 

