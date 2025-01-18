SHELL := $(shell which bash)
.SHELLFLAGS := -eu -o pipefail -c


MAKEFLAGS+=--silent
UID:=$(shell id -u)
DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
PROJECT_DIR:=$(shell dirname ${DOCKER_DIR})
NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo '--runtime nvidia --gpus all' || echo '')
BUILDX_INSTALLED := $(shell docker buildx  1>/dev/null 2>&1 && echo true)
FLAVOR=base
IMAGE=ghcr.io/memristor/mep3

.PHONY: all

all: destroy run

colors:
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)
	$(eval BOLD=\033[1;37m)

vnc:
	$(eval FLAVOR=vnc)
	$(eval IMAGE=ghcr.io/memristor/mep3-vnc)
	true

deploy:
	$(eval FLAVOR=deploy)
	$(eval IMAGE=ghcr.io/memristor/mep3-deploy)
	true

multiple:
	$(eval FLAVOR=multiple)
	true

exists/cmd/%:
	@hash $(*) > /dev/null 2>&1 || (echo "ERROR: '$(*)' must be installed"; exit 1)

exists/env/%:
	@if [ -z '$($(*))' ]; then echo "ERROR: environment variable '$*' not set" && exit 1; fi


build: | exists/cmd/docker colors
	[ ${FLAVOR} == 'multiple' ] && docker buildx bake && exit 0; \
	[ ${FLAVOR} == 'base' ] && docker buildx bake mep3 && exit 0; \
	docker buildx bake mep3-${FLAVOR} && exit 0;

run: | exists/cmd/docker test-nvidia
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

test-nvidia: | exists/cmd/docker
	lspci | grep -qi nvidia && base64 --decode massage | unxz || true
	docker run --rm \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		${IMAGE} | grep -qi 'nvidia' && \
	printf '%b\n' "${RED}Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!\n${NC}" || \
	true

exec: | exists/cmd/docker
	docker exec -it mep3-${FLAVOR} bash

destroy: | exists/cmd/docker
	docker container kill mep3-${FLAVOR} 1>/dev/null || true
	docker container rm -f mep3-${FLAVOR} 1>/dev/null || true

setup-interactive: | exists/cmd/docker colors
	docker exec -it mep3-${FLAVOR} sh -c '/usr/bin/setup.sh --interactive'
	printf '%b\n%b\n' "${GREEN}Interactive setup complete!${NC}" \
		"Run ${BOLD}make exec${NC} or ${BOLD}docker exec -it mep3-${FLAVOR}${NC} to access the container"
