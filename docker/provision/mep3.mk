mep3-prod-checkout:
	mkdir -p /memristor/ros2_ws/src
	git clone --depth 1 https://github.com/memristor/mep3.git /memristor/ros2_ws/src/mep3
	touch /memristor/ros2_ws/src/mep3/mep3_simulation/COLCON_IGNORE

mep3-full-checkout:
	mkdir -p /memristor/ros2_ws/src
	git clone https://github.com/memristor/mep3.git /memristor/ros2_ws/src/mep3

mep3-rosdep:
	sudo -E rosdep init
	sudo -E apt-get install -y python3-vcstool
	cd /memristor/ros2_ws && vcs import src < /memristor/ros2_ws/src/mep3/mep3.repos
	rosdep --rosdistro ${ROS_DISTRO} update
	cd /memristor/ros2_ws && yes | rosdep --rosdistro ${ROS_DISTRO} install -r --from-paths src --ignore-src

mep3-bashrc:
	sudo -E apt-get install -y bc
	echo 'source /opt/ros/${ROS_DISTRO}/local_setup.bash' >> /memristor/.bashrc
	ln -sf /memristor/ros2_ws/src/mep3/docker/config/shortcuts.sh /memristor/.setup/config/shortcuts.sh
	echo 'cd /memristor/ros2_ws' >> /memristor/.bashrc

mep3-symlink-setup:
	ln -sf /memristor/ros2_ws/src/mep3/docker/Makefile /memristor/.setup/Makefile
	rm -rf /memristor/.setup/provision
	ln -sf /memristor/ros2_ws/src/mep3/docker/provision /memristor/.setup/provision
