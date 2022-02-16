NVIDIA_GPU	!= command -v nvidia-smi 1>/dev/null 2>/dev/null && echo '--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all'

docker-prepare-%:
	docker kill mep3-$* || true
	docker rm mep3-$* || true
	docker build . -t mep3 --build-arg user_id=${UID}

docker-run-prod:
	docker run -e PLATFORM=prod \
		--net=host \
		--cap-add SYS_ADMIN \
		--restart unless-stopped \
		--name mep3-prod \
		-d -it mep3

docker-run-devel:
	docker run -e DISPLAY -e PLATFORM=devel \
		-v ${SRC}:/memristor/ros2_ws:rw \
		-v ~/.Xauthority:/memristor/.Xauthority:ro \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-v /dev/dri:/dev/dri:ro ${NVIDIA_GPU} \
		--net=host \
		--cap-add SYS_ADMIN \
		--restart unless-stopped \
		--name mep3-devel \
		-d -it mep3

docker-run-vnc:
	docker run -e DISPLAY=${VNC_HOST_DISPLAY} -e PLATFORM=vnc \
		-v ~/.Xauthority:/memristor/.Xauthority.host:ro \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro ${NVIDIA_GPU} \
		--ipc=host \
		--net=host \
		--cap-add SYS_ADMIN \
		--restart unless-stopped \
		--name mep3-vnc-${VNC_HOST_PORT} \
		-d -it mep3

docker-restart-%:
	docker restart mep3-$*

docker-logs-%:
	docker logs --follow mep3-$*

bash-%:
	@docker exec -it mep3-$* bash
