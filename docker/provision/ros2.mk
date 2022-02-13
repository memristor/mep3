ros-apt:
	sudo curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$(shell dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu ${UBUNTU_CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
	sudo -E apt-get update

ros-base:
	sudo -E apt-get install -y ros-${ROS_DISTRO}-ros-base python3-rosdep build-essential python3-argcomplete python3-colcon-common-extensions

ros-desktop:
	sudo -E apt-get install -y ros-${ROS_DISTRO}-desktop python3-rosdep build-essential python3-argcomplete python3-colcon-common-extensions
