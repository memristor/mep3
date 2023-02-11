#/usr/bin/env bash

# Configure ROS
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade -y

# Install packages
sudo apt install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp \
    avahi-discover \
    can-utils \
    python3-pip \
    vim

wget -O /tmp/diff_drive_controller.deb http://snapshots.ros.org/humble/2022-11-23/ubuntu/pool/main/r/ros-humble-diff-drive-controller/ros-humble-diff-drive-controller_2.12.0-1jammy.20221108.233651_arm64.deb && \
    sudo apt install -y --allow-downgrades /tmp/diff_drive_controller.deb && \
    rm -f /tmp/diff_drive_controller.deb

python3 -m pip install scipy transforms3d

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "test -f /opt/ros/humble/local_setup.bash && source /opt/ros/humble/local_setup.bash" >> ~/.bashrc
echo "test -f ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
