#!/usr/bin/env bash


configure_proxy() {
    if [ "$1" -eq 1 ]; then
        sed '/# Setup_proxy/d' -i /memristor/.bashrc
        sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
    else
        sed '/# Setup_proxy/d' -i /memristor/.bashrc
        sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
        echo "source /memristor/ros2_ws/src/mep3/docker/config/proxy.sh &>/dev/null # Setup_proxy" >> /memristor/.bashrc
        source /memristor/ros2_ws/src/mep3/docker/config/proxy.sh
        echo 'Acquire::http::Proxy "http://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
        echo 'Acquire::https::Proxy "https://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
    fi
}

first_time_setup() {
    clear
    sudo -E rosdep init
    sudo -E apt-get install -y python3-vcstool
    rosdep --rosdistro "${ROS_DISTRO}" update
    cd /memristor/ros2_ws && yes | rosdep --rosdistro "${ROS_DISTRO}" install -r --from-paths src --ignore-src 
}

enhanced_shell_prompt() {
    if [ "$1" -eq 1 ]; then
        sed '/# Setup_prompt/d' -i /memristor/.config/fish/config.fish
        echo 'starship init fish | source # Setup_prompt' >> /memristor/.config/fish/config.fish
        eval "curl -sS https://starship.rs/install.sh | sh -s -- --yes"
    else
        sed '/# Setup_prompt/d' -i /memristor/.config/fish/config.fish
    fi    
}

shell_shortcuts() {
    if [ "$1" -eq 1 ]; then
        sed '/# Setup_shortcuts/d' -i /memristor/.config/fish/config.fish
        echo "source /memristor/ros2_ws/src/mep3/docker/config/fish/shortcuts.fish &>/dev/null # Setup_shortcuts" >> /memristor/.config/fish/config.fish
        echo "source /memristor/ros2_ws/src/mep3/docker/config/fish/git.fish &>/dev/null # Setup_shortcuts" >> /memristor/.config/fish/config.fish
    else
        sed '/# Setup_shortcuts/d' -i /memristor/.config/fish/config.fish
    fi     
}

vnc() {
    if [ "$1" -eq 2 ]; then
        VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
        VNC_CMD="VGL=-vgl DISPLAY=$DISPLAY PORT_VNC=5910 PORT_WEB=6810 ${VNC_CMD}"
        sed '/# Setup_VNC/d' -i /memristor/.bashrc
        echo "$VNC_CMD # Setup_VNC" >> /memristor/.bashrc
    elif [ "$1" -eq 1 ]; then
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
}

if [ "$1" != "default" ] && [ -z "${VENDOR}" ]; then
    dialog --title 'mep3 setup' --default-item '2' --menu 'Setup script vendor:' 0 0 0 1 'Docker image' 2 'Active git branch' 2>/tmp/answer
    if grep -q '^2$' /tmp/answer; then
        VENDOR=1 exec /memristor/ros2_ws/src/mep3/docker/config/setup.sh
        exit
    fi
else
    commit="$(cd /memristor/ros2_ws/src/mep3 && git rev-parse --short HEAD)"
    if [ "$1" == "default" ]; then
        echo "Using staging setup script (${commit})"
    else
        dialog --msgbox "Using staging setup script (${commit})" 5 42
    fi
fi

if [ "$1" == "default" ] || dialog --title 'mep3 config' --defaultno --yesno 'Enable UNS proxy' 5 30; then
    configure_proxy 1
else
    configure_proxy 0
fi

if [ "$1" == "default" ] || dialog --title 'mep3 config' --yesno 'Run first time ROS setup' 5 30; then
    first_time_setup
fi

if [ "$1" == "default" ] || dialog --title 'mep3 config' --yesno 'Enable enhanced shell prompt' 5 38; then
    enhanced_shell_prompt 1
else
    enhanced_shell_prompt 0
fi

if [ "$1" == "default" ] || dialog --title 'mep3 config' --yesno 'Enable shell shortcuts' 5 30; then
    shell_shortcuts 1
else
    shell_shortcuts 0
fi

if test -f /opt/TurboVNC/bin/vncserver; then
    if [ "$1" == "default" ] || dialog --title 'mep3 config' --yesno 'Enable VNC' 5 30; then
        if [ "$1" == "default" ]; then
        vnc 2
        else
        vnc 1
        fi
    else
        vnc 0
    fi
fi

sed '/# Setup_shell/d' -i /memristor/.bashrc
echo 'echo "$-" | grep i -q && exec fish # Setup_shell' >> /memristor/.bashrc

clear
