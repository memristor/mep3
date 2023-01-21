UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
USBTTY_MOUNT:=$(shell find /dev -type c -name 'ttyUSB*' -printf ' -v %p:%p:rw ')
FLAVOR=devel
IMAGE=mep3

.PHONY: all

all: destroy build run setup exec

vnc:
	$(eval FLAVOR=vnc)
	$(eval IMAGE=mep3-vnc)
	@true

build:
	@DOCKER_BUILDKIT=1 docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.base -t mep3 --build-arg UID=${UID}
	@[ ${FLAVOR} != 'devel' ] && \
		DOCKER_BUILDKIT=1 docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.${FLAVOR} -t ${IMAGE} || \
		true

usb-tty:
	@docker exec mep3-${FLAVOR} sh -c \
		'find /dev -name ttyUSB* -exec sudo chown root:dialout {} \;'

run:
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
		-v /dev/bus/usb:/dev/bus/usb:ro ${USBTTY_MOUNT} \
		-v ${ROOT_DIR}/../..:/memristor/ros2_ws/src:rw \
		-d -it ${IMAGE}

exec: usb-tty
	@docker exec -it mep3-${FLAVOR} bash

destroy:
	@docker container kill mep3-${FLAVOR} || true
	@docker container rm -f mep3-${FLAVOR} || true

setup:
	@docker exec -it mep3-${FLAVOR} /usr/bin/setup.sh
	@echo 'Restarting mep3-${FLAVOR} container...'
	@docker restart mep3-${FLAVOR}
