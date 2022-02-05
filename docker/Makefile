.PHONY: devel

UBUNTU_CODENAME	!=	. /etc/os-release && echo $$UBUNTU_CODENAME
ROS_DISTRO	=	galactic
SRC		!=	realpath ./../../../
UID		!=	id -u

prod: docker-prepare-prod docker-run-prod docker-logs-prod
devel: docker-prepare-devel docker-run-devel docker-logs-devel

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

docker-logs-%:
	docker logs --follow mep3-$*

bash-%:
	@docker exec -it mep3-$* bash

provision-prod: ros-apt ros-base mep3-prod-checkout mep3-rosdep mep3-bashrc done
provision-devel: ros-apt ros-desktop mep3-rosdep mep3-bashrc webots groot vscode done

ros-apt:
	sudo curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(shell dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu ${UBUNTU_CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
	sudo apt update

ros-base:
	sudo apt-get install -y ros-${ROS_DISTRO}-ros-base python3-rosdep build-essential python3-argcomplete python3-colcon-common-extensions

ros-desktop:
	sudo apt-get install -y ros-${ROS_DISTRO}-desktop python3-rosdep build-essential python3-argcomplete python3-colcon-common-extensions

webots:
	wget -nv -O ./webots.deb 'https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb'
	sudo apt-get install -y ./webots.deb
	rm -f ./webots.deb

groot:
	wget -nv -O ./Groot.AppImage 'https://github.com/BehaviorTree/Groot/releases/download/1.0.0/Groot-1.0.0-x86_64.AppImage'
	chmod +x ./Groot.AppImage
	./Groot.AppImage --appimage-extract
	rm -f ./Groot.AppImage
	sudo mv ./squashfs-root /opt/groot
	find /opt/groot -type d -exec sudo chmod 755 {} \;
	sudo ln -sf /opt/groot/AppRun /usr/bin/groot
	sudo chmod +x /usr/bin/groot
	sudo apt-get install -y mesa-utils libharfbuzz0b libgtk-3-dev

vscode:
	wget -nv -O ./vscode.deb 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64'
	sudo apt-get install -y ./vscode.deb
	rm -f ./vscode.deb
	sudo apt-get install -y alsa libxshmfence1 libgtk-3-dev
	test -d /memristor/ros2_ws/.vscode || rsync -rP ./.vscode/ /memristor/ros2_ws/.vscode/
	code --install-extension CoenraadS.bracket-pair-colorizer-2
	code --install-extension eamodio.gitlens
	code --install-extension ms-python.python
	code --install-extension ms-vscode.cpptools
	code --install-extension twxs.cmake
	code --install-extension usernamehw.errorlens

mep3-prod-checkout:
	mkdir -p /memristor/ros2_ws/src
	git clone --depth 1 https://github.com/memristor/mep3.git /memristor/ros2_ws/src/mep3
	touch /memristor/ros2_ws/src/mep3/mep3_simulation/COLCON_IGNORE

mep3-rosdep:
	sudo rosdep init
	rosdep --rosdistro ${ROS_DISTRO} update
	cd /memristor/ros2_ws && yes | rosdep --rosdistro ${ROS_DISTRO} install --from-paths src --ignore-src

mep3-bashrc:
	echo 'source /opt/ros/${ROS_DISTRO}/local_setup.bash' >> /memristor/.bashrc
	echo 'cd /memristor/ros2_ws' >> /memristor/.bashrc

done:
	touch /memristor/.setup/completed
	@echo "$(tput bold; tput setaf 2)=============================================$(tput sgr0)"
	@echo "$(tput bold; tput setaf 2)Container provisioning completed successfully$(tput sgr0)"
