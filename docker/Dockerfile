FROM ubuntu:20.04

# Platform configuration: ros-base / desktop
ARG hw_platform=ros-base
ARG DEBIAN_FRONTEND=noninteractive

# Update apt cache
RUN apt-get update

# System locale
RUN echo 'Europe/Belgrade' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Europe/Belgrade /etc/localtime && \
    apt-get install -y --no-install-recommends tzdata locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# General dependencies
RUN apt-get -y install apt-utils gnupg2 lsb-release git git-lfs wget curl sudo rsync python3 python3-pip

# User setup
RUN useradd -d /memristor -m \
            -u 1000 -U \
            -s /usr/bin/bash \
            -c "Memristor Robotics" memristor && \
    echo 'memristor ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    echo 'export PS1="$(tput setaf 4)\w$(tput sgr0) [$(tput setaf 5)\$?$(tput sgr0)] $(tput setaf 3)\$(git branch --show-current 2>/dev/null | xargs -I{} echo \"({})\")$(tput sgr0)\n$(tput setaf 2)\\\\$ \\[$(tput sgr0)\\]"' | tee -a /memristor/.bashrc /root/.bashrc



# Container startup
USER memristor
WORKDIR /memristor
ENTRYPOINT bash
