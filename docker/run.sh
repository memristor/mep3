#!/bin/sh

dir="$(CDPATH="$(cd -- "$(dirname -- "$0")")" && pwd)"
src="${1:-"$(realpath "$dir/../../../")"}"

docker build "$dir" --build-arg hw_platform=desktop -t mep3 && \
docker run -e DISPLAY \
           --cap-add SYS_ADMIN \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -v "$src":/memristor/ros2_ws \
           -it mep3
