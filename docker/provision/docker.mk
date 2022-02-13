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
		-v /dev/dri:/dev/dri:ro \
		--net=host \
		--cap-add SYS_ADMIN \
		--restart unless-stopped \
		--name mep3-devel \
		-d -it mep3

docker-run-vnc:
	docker run -e PLATFORM=vnc \
		-v /dev/dri:/dev/dri:ro \
		--publish=${VNC_HOST_PORT}:5901 \
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
