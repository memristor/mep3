.PHONY: prod

UBUNTU_CODENAME	!=	. /etc/os-release && echo $$UBUNTU_CODENAME
ROS_DISTRO		=	galactic

prod: ros-apt ros-base mep3-prod-checkout mep3-rosdep mep3-bashrc done
devel: ros-apt ros-desktop mep3-rosdep mep3-bashrc webots groot done

ros-apt:
	sudo curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(shell dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu ${UBUNTU_CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
	sudo apt update

ros-base:
	sudo apt-get install -y ros-${ROS_DISTRO}-ros-base python3-rosdep build-essential python3-argcomplete python3-colcon-common-extensions

ros-desktop:
	sudo apt-get install -y ros-${ROS_DISTRO}-desktop python3-rosdep build-essential python3-argcomplete python3-colcon-common-extensions

webots:
	wget -nv -O ./webots.deb https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb
	sudo apt-get install -y ./webots.deb
	rm -f ./webots.deb

groot:
	wget -nv -O ./Groot.AppImage https://github.com/BehaviorTree/Groot/releases/download/1.0.0/Groot-1.0.0-x86_64.AppImage
	chmod +x ./Groot.AppImage
	./Groot.AppImage --appimage-extract
	rm -f ./Groot.AppImage
	sudo mv ./squashfs-root /opt/groot
	find /opt/groot -type d -exec sudo chmod 755 {} \;
	sudo ln -sf /opt/groot/AppRun /usr/bin/groot
	sudo chmod +x /usr/bin/groot
	sudo apt-get install -y mesa-utils libharfbuzz0b libgtk-3-dev

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
	@echo "$(tput bold; tput setaf 2)=============================================$(tput sgr0)"
	@echo "$(tput bold; tput setaf 2)Container provisioning completed successfully$(tput sgr0)"
