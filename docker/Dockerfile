FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ARG UID=1000

# User
RUN useradd -d /memristor -m \
            -u $UID -U \
            -s /usr/bin/bash \
            -c "Memristor Robotics" memristor && \
    echo "memristor ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# System utilities
RUN apt-get update
RUN apt-get install -y \
        git \
        git-lfs \
        curl \
        wget \
        vim \
        rsync \
        dialog

# VS Code
RUN curl -L -o /tmp/vscode.deb \
        'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64' && \
    apt-get install -y /tmp/vscode.deb && \
    rm -f /tmp/vscode.deb
RUN apt-get install -y alsa libxshmfence1 libgtk-3-dev
RUN code --user-data-dir /memristor --install-extension eamodio.gitlens && \
    code --user-data-dir /memristor --install-extension ms-python.python && \
    code --user-data-dir /memristor --install-extension ms-vscode.cpptools-extension-pack && \
    code --user-data-dir /memristor --install-extension usernamehw.errorlens && \
    code --user-data-dir /memristor --install-extension ms-iot.vscode-ros

# Webots
RUN curl -L -o /tmp/webots.deb \
        'https://github.com/cyberbotics/webots/releases/download/R2022b/webots_2022b_amd64.deb' && \
    apt-get install -y /tmp/webots.deb && \
    rm -f /tmp/webots.deb && \
    mkdir -p /memristor/.config/Cyberbotics

# TurboVNC
RUN curl -L -o /tmp/turbovnc.deb \
        'https://downloads.sourceforge.net/project/turbovnc/3.0.1/turbovnc_3.0.1_amd64.deb?use_mirror=autoselect' && \
    apt-get install -y /tmp/turbovnc.deb && \
    rm -f /tmp/turbovnc.deb && \
	echo 'PATH="$PATH:/opt/TurboVNC/bin"' >> /memristor/.bashrc

# VirtualGL
RUN curl -L -o /tmp/virtualgl.deb \
        'https://downloads.sourceforge.net/project/virtualgl/3.0.2/virtualgl_3.0.2_amd64.deb?use_mirror=autoselect' && \
    apt-get install -y /tmp/virtualgl.deb && \
    rm -f /tmp/virtualgl.deb && \
	echo 'PATH="$PATH:/opt/VirtualGL/bin"' >> /memristor/.bashrc && \
	printf "1\n\n\n\nX" | /opt/VirtualGL/bin/vglserver_config && \
	sudo usermod -a -G vglusers memristor

# NoVNC
RUN apt-get install -y novnc && \
	ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html && \
    mkdir -p /memristor/.host

# XFCE
RUN apt-get install -y --no-install-recommends \
        xorg \
        dbus-x11 \
        xfce4 \
        xfce4-terminal \
        firefox && \
    echo '/usr/sbin/lightdm' > /etc/X11/default-display-manager

# ROS packages
RUN apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-rviz2 \
        ros-humble-teleop-twist-keyboard \
        ros-humble-webots-ros2
RUN echo 'source /opt/ros/humble/local_setup.bash' >> /memristor/.bashrc

# User config
RUN echo 'export PS1="$(tput setaf 4)\w$(tput sgr0) [$(tput setaf 5)\$?$(tput sgr0)] $(tput setaf 3)\$(git branch --show-current 2>/dev/null | xargs -I{} echo \"({})\")$(tput sgr0)\n$(tput setaf 2)\\$ \\[$(tput sgr0)\\]"' >> /memristor/.bashrc
RUN echo 'cp -p /memristor/.host/.Xauthority /memristor/.Xauthority' >> /memristor/.bashrc
COPY ./config/vscode/. /memristor/ros2_ws/.vscode/
COPY ./config/Cyberobotics/. /memristor/.config/Cyberbotics/
COPY ./config/setup.sh /usr/bin/
COPY ./config/vnc.sh /usr/bin/

RUN chown -R memristor:memristor /memristor

USER memristor
WORKDIR /memristor/ros2_ws
