#!/bin/sh

dir="$(CDPATH="$(cd -- "$(dirname -- "$0")")" && pwd)"
src="${1:-"$(realpath "$dir/../../../")"}"

while true; do
    printf 'Platform (prod or devel): '
    read -r PLATFORM
    case "$PLATFORM" in
        prod)
            WS_MOUNT=''
            break;;
        devel)
            WS_MOUNT="-v $src:/memristor/ros2_ws:rw"
            break;;
        *)
            echo 'Invalid input!';;
    esac
done

export PLATFORM

docker kill "mep3-$PLATFORM"
docker rm "mep3-$PLATFORM"
docker build "$dir" -t mep3 --build-arg user_id="$(id -u)" && \
docker run -e DISPLAY -e PLATFORM \
              $WS_MOUNT \
           -v "$HOME"/.Xauthority:/memristor/.Xauthority:ro \
           -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           -v /dev/dri:/dev/dri:ro \
           --net=host \
           --cap-add SYS_ADMIN \
           --restart unless-stopped \
           --name "mep3-$PLATFORM" \
           -d -it mep3

docker logs --follow "mep3-$PLATFORM"
