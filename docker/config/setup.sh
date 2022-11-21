#!/bin/sh

if dialog --title 'mep3 config' --yesno 'Run first time ROS setup' 5 30; then
    clear
    sudo -E rosdep init
    sudo -E apt-get install -y python3-vcstool
    cd /memristor/ros2_ws && vcs import --recursive src < /memristor/ros2_ws/src/mep3/mep3.repos
	rosdep --rosdistro ${ROS_DISTRO} update
	cd /memristor/ros2_ws && yes | rosdep --rosdistro ${ROS_DISTRO} install -r --from-paths src --ignore-src
fi

if dialog --title 'mep3 config' --yesno 'Enable VNC' 5 30; then
    VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
    if dialog --title 'mep3 config' --yesno 'Enable GPU acceleration' 5 30; then
        VNC_CMD="VGL=-vgl ${VNC_CMD}"
    fi
    if dialog --title 'mep3 config' --inputbox 'Enter host X11 display server location' 8 30 "$DISPLAY" 2>/tmp/answer; then
        VNC_CMD=" DISPLAY=$(cat /tmp/answer) ${VNC_CMD}"
    fi
    sed '/# Setup_VNC/d' -i /memristor/.bashrc
    echo "$VNC_CMD # Setup_VNC" >> /memristor/.bashrc
else
    sed '/# Setup_VNC/d' -i /memristor/.bashrc
fi

clear
