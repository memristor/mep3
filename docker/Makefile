SHELL := $(shell which bash)
.SHELLFLAGS := -eu -o pipefail -c


MAKEFLAGS+=--silent
export UID:=$(shell id -u)
export DOCKER_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
export PROJECT_DIR:=$(shell dirname ${DOCKER_DIR})
export NVIDIA_GPU:=$(shell (docker info | grep Runtimes | grep nvidia 1> /dev/null && command -v nvidia-smi 1>/dev/null 2>/dev/null && nvidia-smi | grep Processes 1>/dev/null 2>/dev/null) && echo 'true' || echo '')
export FLAVOR=base
export IMAGE=ghcr.io/memristor/mep3

colors:
	$(eval NC=\033[1;0m)
	$(eval RED=\033[1;31m)
	$(eval GREEN=\033[1;32m)
	$(eval BOLD=\033[1;37m)

vnc:
	$(eval FLAVOR=vnc)
	$(eval IMAGE=${IMAGE}-vnc)
	true

multiple:
	$(eval FLAVOR=multiple)
	true

exists/cmd/%:
	hash $(*) > /dev/null 2>&1 || (echo "ERROR: '$(*)' must be installed"; exit 1)

build: | exists/cmd/docker
	[ ${FLAVOR} == 'multiple' ] && docker buildx bake && exit 0; \
	[ ${FLAVOR} == 'base' ] && docker buildx bake mep3 && exit 0; \
	docker buildx bake mep3-${FLAVOR} && exit 0; \

test-nvidia: | exists/cmd/docker
	lspci | grep -qi nvidia && base64 --decode massage | unxz || true
	docker run --rm \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		${IMAGE} | grep -qi 'nvidia' && \
	printf '%b\n' "${RED}Detected NVIDIA GPU in system, but missing packets, look up NVIDIA GPU section in README!\n${NC}" || \
	true

run: | exists/cmd/docker test-nvidia
	if [ -n "${NVIDIA_GPU}" ]; then \
		docker compose up -d mep3-${FLAVOR}-nvidia; \
	else \
		docker compose up -d mep3-${FLAVOR}; \
	fi

destroy: | exists/cmd/docker
	docker compose kill --remove-orphans || true
	docker compose down --remove-orphans

exec: | exists/cmd/docker
	docker exec -it mep3-${FLAVOR} bash

setup-interactive: | exists/cmd/docker colors
	docker exec -it mep3-${FLAVOR} sh -c '/usr/bin/setup.sh --interactive'
	printf '%b\n%b\n' "${GREEN}Interactive setup complete!${NC}" \
		"Run ${BOLD}make exec${NC} or ${BOLD}docker exec -it mep3-${FLAVOR}${NC} to access the container"
