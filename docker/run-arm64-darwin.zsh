export DOCKER_COMPOSE_FILE="docker/compose/docker-compose.humble-base.arm64.darwin.yml"

# fast run
# docker compose -f ${DOCKER_COMPOSE_FILE} up --build
docker compose -f ${DOCKER_COMPOSE_FILE} run --build ros bash


# rm
# sudo docker-compose rm -f ${DOCKER_COMPOSE_FILE} 
