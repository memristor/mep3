mep3-prod-checkout:
	mkdir -p /memristor/ros2_ws/src
	git clone --depth 1 https://github.com/memristor/mep3.git /memristor/ros2_ws/src/mep3
	touch /memristor/ros2_ws/src/mep3/mep3_simulation/COLCON_IGNORE

mep3-full-checkout:
	mkdir -p /memristor/ros2_ws/src
	git clone https://github.com/memristor/mep3.git /memristor/ros2_ws/src/mep3

mep3-rosdep:
	sudo -E rosdep init
	rosdep --rosdistro ${ROS_DISTRO} update
	cd /memristor/ros2_ws && yes | rosdep --rosdistro ${ROS_DISTRO} install --from-paths src --ignore-src

mep3-bashrc:
	echo 'source /opt/ros/${ROS_DISTRO}/local_setup.bash' >> /memristor/.bashrc
	ln -sf /memristor/ros2_ws/src/mep3/docker/config/shortcuts.sh /memristor/.setup/config/shortcuts.sh
	echo 'cd /memristor/ros2_ws' >> /memristor/.bashrc
