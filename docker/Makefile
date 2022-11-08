UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
NVIDIA_GPU:=$(shell docker info | grep Runtimes | grep nvidia 1> /dev/null && echo '--runtime nvidia --gpus all' || echo '')

.PHONY: destroy build run setup

build:
	@docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.base -t mep3 --build-arg UID=${UID}

build-vnc: build
	@docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.vnc -t mep3-vnc

run:
	@docker run \
		--net=host \
		--ipc=host \
		--cap-add SYS_ADMIN \
		--name mep3-devel \
		--privileged \
		--restart unless-stopped \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/memristor/.host/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro \
		-v /dev/bus/usb:/dev/bus/usb:ro \
		-v ${ROOT_DIR}/../..:/memristor/ros2_ws/src:rw \
		-d -it mep3

exec:
	@docker exec -it mep3-devel bash

destroy:
	@docker container kill mep3-devel || true
	@docker container rm -f mep3-devel || true

setup:
	@docker exec -it mep3-devel /usr/bin/setup.sh
	@echo 'Restarting mep3-devel container...'
	@docker restart mep3-devel
