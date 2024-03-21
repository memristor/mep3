MAKEFLAGS+=--silent
UID:=$(shell id -u)
DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
PROJECT_DIR:=$(shell dirname ${DOCKER_DIR})
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
FLAVOR=devel
IMAGE=ghcr.io/memristor/mep3

.PHONY: all

all: destroy run setup-default

colors:
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)
	$(eval BOLD=\033[1;37m)

vnc:
	$(eval FLAVOR=vnc)
	$(eval IMAGE=ghcr.io/memristor/mep3-vnc)
	true

build:
	echo ${NO_CACHE_ARG}
	DOCKER_BUILDKIT=1 docker build ${DOCKER_DIR} -f ${DOCKER_DIR}/Dockerfile.base -t mep3 ${DOCKER_ARGS} --build-arg UID=${UID}
	[ ${FLAVOR} != 'devel' ] && \
		DOCKER_BUILDKIT=1 docker build ${DOCKER_DIR} -f ${DOCKER_DIR}/Dockerfile.${FLAVOR} -t ${IMAGE} ${DOCKER_ARGS} || \
		true

run: test-nvidia
	docker run \
		--net=host \
		--ipc=host \
		--cap-add SYS_ADMIN \
		--name mep3-${FLAVOR} \
		--privileged \
		--restart unless-stopped \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/memristor/.host/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro \
		-v /dev:/dev:rw \
		-v ${PROJECT_DIR}:/memristor/ros2_ws/src/mep3:rw \
		-d -it ${IMAGE} 1>/dev/null

test-nvidia: colors
	lspci | grep -qi nvidia && base64 --decode massage | unxz || true
	docker run --rm \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		${IMAGE} | grep -qi 'nvidia' && \
	printf '%b\n' "${RED}Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!\n${NC}" || \
	true

start-code-server:
	docker exec -d -it mep3-${FLAVOR} bash -c 'pgrep code-server || code-server /memristor/ros2_ws/src/mep3' && \
	xdg-open 'localhost:31415?folder=/memristor/ros2_ws/src/mep3'

stop-code-server:
	docker exec -it mep3-${FLAVOR} pkill -f code-server

exec:
	docker exec -it mep3-${FLAVOR} bash

destroy:
	docker container kill mep3-${FLAVOR} 1>/dev/null || true
	docker container rm -f mep3-${FLAVOR} 1>/dev/null || true

setup-default: colors
	docker exec -it mep3-${FLAVOR} sh -c '/usr/bin/setup.sh --default'
	printf '%b\n%b\n' "${GREEN}Default setup complete!${NC}" \
		"Run ${BOLD}make exec${NC} or ${BOLD}docker exec -it mep3-${FLAVOR}${NC} to access the container"

setup-interactive: colors
	docker exec -it mep3-${FLAVOR} sh -c '/usr/bin/setup.sh --interactive'
	printf '%b\n%b\n' "${GREEN}Interactive setup complete!${NC}" \
		"Run ${BOLD}make exec${NC} or ${BOLD}docker exec -it mep3-${FLAVOR}${NC} to access the container"
