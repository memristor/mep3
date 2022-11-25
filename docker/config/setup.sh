#!/bin/sh

if [ -z "${VENDOR}" ]; then
    dialog --title 'mep3 setup' --default-item '0' --menu 'Setup script vendor:' 0 0 0 1 'Docker image' 2 'Active git branch' 2>/tmp/answer
    if grep -q '^2$' /tmp/answer; then
        VENDOR=1 exec /memristor/ros2_ws/src/mep3/docker/config/setup.sh
        exit
    fi
else
    commit="$(cd /memristor/ros2_ws/src/mep3 && git rev-parse --short HEAD)"
    dialog --msgbox "Using staging setup script (${commit})" 5 42
fi

if dialog --title 'mep3 config' --yesno 'Run first time ROS setup' 5 30; then
    clear
    sudo -E rosdep init
    sudo -E apt-get install -y python3-vcstool
    cd /memristor/ros2_ws && vcs import src < /memristor/ros2_ws/src/mep3/mep3.repos
	rosdep --rosdistro "${ROS_DISTRO}" update
	cd /memristor/ros2_ws && yes | rosdep --rosdistro "${ROS_DISTRO}" install -r --from-paths src --ignore-src
fi

if dialog --title 'mep3 config' --yesno 'Auto-source default ROS workspace' 5 38; then
    sed '/# Setup_default_workspace/d' -i /memristor/.bashrc
    echo "source /memristor/ros2_ws/install/local_setup.bash &>/dev/null # Setup_default_workspace" >> /memristor/.bashrc
else
    sed '/# Setup_default_workspace/d' -i /memristor/.bashrc
fi

if dialog --title 'mep3 config' --yesno 'Enable shell shortcuts' 5 30; then
    sed '/# Setup_shortcuts/d' -i /memristor/.bashrc
    echo "source /memristor/ros2_ws/src/mep3/docker/config/shortcuts.sh &>/dev/null # Setup_shortcuts" >> /memristor/.bashrc
else
    sed '/# Setup_shortcuts/d' -i /memristor/.bashrc
fi

if test -f /opt/TurboVNC/bin/vncserver; then
    if dialog --title 'mep3 config' --yesno 'Enable VNC' 5 30; then
        VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
            if dialog --title 'mep3 config' --yesno 'Enable GPU acceleration' 5 30; then
            VNC_CMD="VGL=-vgl ${VNC_CMD}"
        fi
        if dialog --title 'mep3 config' --inputbox 'Enter host X11 display server location' 8 30 "$DISPLAY" 2>/tmp/answer; then
            VNC_CMD=" DISPLAY=$(cat /tmp/answer) ${VNC_CMD}"
        fi
        if dialog --title 'mep3 config' --inputbox 'Enter VNC server port' 8 30 5910 2>/tmp/answer; then
            VNC_CMD=" PORT_VNC=$(cat /tmp/answer) ${VNC_CMD}"
        fi
        if dialog --title 'mep3 config' --inputbox 'Enter noVNC webserver port' 8 30 6810 2>/tmp/answer; then
            VNC_CMD=" PORT_WEB=$(cat /tmp/answer) ${VNC_CMD}"
        fi
        sed '/# Setup_VNC/d' -i /memristor/.bashrc
        echo "$VNC_CMD # Setup_VNC" >> /memristor/.bashrc
    else
        sed '/# Setup_VNC/d' -i /memristor/.bashrc
    fi
fi

clear
