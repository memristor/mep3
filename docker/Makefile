UID:=$(shell id -u)
DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
PROJECT_DIR:=$(shell dirname ${DOCKER_DIR})
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
FLAVOR=devel
IMAGE=ghcr.io/memristor/mep3

.PHONY: all

all: destroy test-nvidia run setup-default exec start-code-server

vnc:
	$(eval FLAVOR=vnc)
	$(eval IMAGE=ghcr.io/memristor/mep3-vnc)
	@true

run:
	@${MAKE} -s test-nvidia
	@docker run \
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
		-d -it ${IMAGE}

test-nvidia:	
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)

	@lspci | grep -qi nvidia && base64 --decode massage | unxz || true
	
	@docker run --rm \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		${IMAGE} | grep -qi 'nvidia' && \
	printf '%b\n' "${RED}Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!\n${NC}" || \
	printf '%b\n' "${GREEN}Everything set up!${NC}"

start-code-server:
	@docker exec -d -it mep3-${FLAVOR} bash -c 'pgrep code-server || code-server /memristor/ros2_ws/src/mep3' && \
	xdg-open 'localhost:31415?folder=/memristor/ros2_ws/src/mep3'

stop-code-server:
	@docker exec -it mep3-${FLAVOR} pkill -f code-server

exec:
	@docker exec -it mep3-${FLAVOR} bash

destroy:
	@docker container kill mep3-${FLAVOR} || true
	@docker container rm -f mep3-${FLAVOR} || true

setup-default:
	@docker exec -it mep3-${FLAVOR} sh -c '/usr/bin/setup.sh --all' 
	@echo 'Restarting mep3-${FLAVOR} container...'
	@docker restart mep3-${FLAVOR}

setup-interactive:
	@docker exec -it mep3-${FLAVOR} sh -c '/usr/bin/setup.sh --all --interactive' 
	@echo 'Restarting mep3-${FLAVOR} container...'
	@docker restart mep3-${FLAVOR}
