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
    avahi-discover \
    can-utils \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
    ros-humble-dynamixel-sdk \
    ros-humble-can-msgs \
    ros-humble-ruckig \
    ros-humble-laser-filters \
    ros-humble-domain-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-dynamixel-workbench-toolbox \
    vim

python3 -m pip install scipy transforms3d

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "test -f /opt/ros/humble/local_setup.bash && source /opt/ros/humble/local_setup.bash" >> ~/.bashrc
echo "test -f ~/ros2_ws/install/local_setup.bash && source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
