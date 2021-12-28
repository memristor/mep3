#!/bin/bash
#
# Usage: ./script [rpi|pc]
# Example: ./script pc

# Configure the script
set -e

# Export arguments
TARGET_HARDWARE="$1"
if [ "${TARGET_HARDWARE}" != "rpi" ] && [ "${TARGET_HARDWARE}" != "pc" ]; then
    echo "Usage: ./script [rpi|pc]"
    exit 1
fi

# Install ROS 2
# Referece: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
[ "${TARGET_HARDWARE}" = "rpi" ] && sudo apt install ros-foxy-ros-base
[ "${TARGET_HARDWARE}" = "pc" ] && sudo apt install ros-foxy-desktop

# Install apt dependencies
sudo apt install git build-essential python3-argcomplete python3-colcon-common-extensions

# Clone mep3
git lfs install
mkdir -p ~/foxy_ws/src
git clone https://github.com/memristor/mep3.git ~/foxy_ws/src/mep3
[ "${TARGET_HARDWARE}" = "rpi" ] && touch ~/foxy_ws/src/mep3/mep3_simulation/COLCON_IGNORE

# Build mep3
pushd ~/foxy_ws
    source /opt/ros/foxy/local_setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src
    colcon build
    source ./install/local_setup.bash
popd
