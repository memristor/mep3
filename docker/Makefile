UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
NVIDIA_GPU:=$(shell docker info | grep Runtimes | grep nvidia 1> /dev/null && echo '--runtime nvidia --gpus all' || echo '')
FLAVOR=devel
IMAGE=mep3

.PHONY: destroy build run setup

vnc:
	$(eval FLAVOR=vnc)
	$(eval IMAGE=mep3-vnc)
	@true

build:
	@docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.base -t mep3 --build-arg UID=${UID}
	@[ ${FLAVOR} != 'devel' ] && \
		docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.${FLAVOR} -t ${IMAGE} || \
		true

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
		-v /dev/bus/usb:/dev/bus/usb:ro \
		-v ${ROOT_DIR}/../..:/memristor/ros2_ws/src:rw \
		-d -it ${IMAGE}

exec:
	@docker exec -it mep3-${FLAVOR} bash

destroy:
	@docker container kill mep3-${FLAVOR} || true
	@docker container rm -f mep3-${FLAVOR} || true

setup:
	@docker exec -it mep3-${FLAVOR} /usr/bin/setup.sh
	@echo 'Restarting mep3-${FLAVOR} container...'
	@docker restart mep3-${FLAVOR}
