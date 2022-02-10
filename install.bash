#!/usr/bin/env bash

# Usage: ./script [rpi|pc]
# Example: ./script pc

# Configure the script
set -e

# If there is a ROS environment already sourced it might cause installation issues.
if [ "$(env | grep ROS)" ] && [ ! "$(env | grep 'ROS_DISTRO=galactic')" ]; then
    echo "ERROR: You already have a ROS environment sourced (please check your ~/.bashrc)."
    exit 2
fi

# Export arguments
TARGET_HARDWARE="$1"
if [ "${TARGET_HARDWARE}" != "rpi" ] && [ "${TARGET_HARDWARE}" != "pc" ]; then
    echo "Usage: ./script [rpi|pc]"
    exit 1
fi

# Install ROS 2
# Referece: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2-testing/ubuntu $(source /etc/os-release && echo "$UBUNTU_CODENAME") main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update

case "${TARGET_HARDWARE}" in
    'pc') sudo apt install -y ros-galactic-desktop python3-rosdep;;
    'rpi') sudo apt install -y ros-galactic-ros-base python3-rosdep;;
esac

# Install apt dependencies
sudo apt install -y git build-essential python3-argcomplete python3-colcon-common-extensions

# Clone mep3
mkdir -p ~/galactic_ws/src
git clone https://github.com/memristor/mep3.git ~/galactic_ws/src/mep3
if [ "${TARGET_HARDWARE}" = "rpi" ]; then
    touch ~/galactic_ws/src/mep3/mep3_simulation/COLCON_IGNORE
fi

# Build mep3
pushd ~/galactic_ws
source /opt/ros/galactic/local_setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    # rosdep can be initialized only once so we have to check whether it has already been initialized.
    sudo rosdep init
fi
rosdep update
yes | rosdep install --from-paths src --ignore-src
colcon build
source ./install/local_setup.bash
popd
