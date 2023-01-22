#!/usr/bin/env bash

if [ -z "${VENDOR}" ]; then
    dialog --title 'mep3 setup' --default-item '2' --menu 'Setup script vendor:' 0 0 0 1 'Docker image' 2 'Active git branch' 2>/tmp/answer
    if grep -q '^2$' /tmp/answer; then
        VENDOR=1 exec /memristor/ros2_ws/src/mep3/docker/config/setup.sh
        exit
    fi
else
    commit="$(cd /memristor/ros2_ws/src/mep3 && git rev-parse --short HEAD)"
    dialog --msgbox "Using staging setup script (${commit})" 5 42
fi

if dialog --title 'mep3 config' --defaultno --yesno 'Enable UNS proxy'  5 30; then
    sed '/# Setup_proxy/d' -i /memristor/.bashrc
    sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
    echo "source /memristor/ros2_ws/src/mep3/docker/config/proxy.sh &>/dev/null # Setup_proxy" >> /memristor/.bashrc
    source /memristor/ros2_ws/src/mep3/docker/config/proxy.sh
    echo 'Acquire::http::Proxy "http://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
    echo 'Acquire::https::Proxy "https://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
else
    sed '/# Setup_proxy/d' -i /memristor/.bashrc
    sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
fi

if dialog --title 'mep3 config' --yesno 'Run first time ROS setup' 5 30; then
    clear
    sudo -E rosdep init
    sudo -E apt-get install -y python3-vcstool
    cd /memristor/ros2_ws && vcs import --recursive src < /memristor/ros2_ws/src/mep3/mep3.repos
	  rosdep --rosdistro "${ROS_DISTRO}" update
	  cd /memristor/ros2_ws && yes | rosdep --rosdistro "${ROS_DISTRO}" install -r --from-paths src --ignore-src
fi

if dialog --title 'mep3 config' --defaultno --yesno 'Enable enhanced shell prompt' 5 38; then
    sed '/# Setup_prompt/d' -i /memristor/.config/fish/config.fish
    echo 'starship init fish | source # Setup_prompt' >> /memristor/.config/fish/config.fish
    eval "curl -sS https://starship.rs/install.sh | sh -s -- --yes"
else
    sed '/# Setup_prompt/d' -i /memristor/.config/fish/config.fish
fi

if dialog --title 'mep3 config' --defaultno --yesno 'Enable shell shortcuts' 5 30; then
    sed '/# Setup_shortcuts/d' -i /memristor/.config/fish/config.fish
    echo "source /memristor/ros2_ws/src/mep3/docker/config/fish/shortcuts.fish &>/dev/null # Setup_shortcuts" >> /memristor/.config/fish/config.fish
    echo "source /memristor/ros2_ws/src/mep3/docker/config/fish/git.fish &>/dev/null # Setup_shortcuts" >> /memristor/.config/fish/config.fish
    dialog --msgbox 'Type "h" into terminal to list all mep3 shortcuts' 5 53
else
    sed '/# Setup_shortcuts/d' -i /memristor/.config/fish/config.fish
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

sed '/# Setup_shell/d' -i /memristor/.bashrc
echo 'echo "$-" | grep i -q && exec fish # Setup_shell' >> /memristor/.bashrc

clear
