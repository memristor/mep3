FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
ARG user_id=1000

# Update apt cache
RUN apt-get update

# System locale
RUN echo 'Europe/Belgrade' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Europe/Belgrade /etc/localtime && \
    apt-get install -y --no-install-recommends tzdata locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    printf 'XKBMODEL="pc105"\nXKBLAYOUT="us"\nXKBVARIANT="intl"\nXKBOPTIONS=""\n' > /etc/default/keyboard

# General dependencies
RUN apt-get -y install apt-utils gnupg2 lsb-release git git-lfs wget curl sudo rsync python3 python3-pip keyboard-configuration

# User setup
RUN useradd -d /memristor -m \
            -u $user_id -U \
            -s /usr/bin/bash \
            -c "Memristor Robotics" memristor && \
    echo 'memristor ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    echo 'export PS1="$(tput setaf 4)\w$(tput sgr0) [$(tput setaf 5)\$?$(tput sgr0)] $(tput setaf 3)\$(git branch --show-current 2>/dev/null | xargs -I{} echo \"({})\")$(tput sgr0)\n$(tput setaf 2)\\\\$ \\[$(tput sgr0)\\]"' | tee -a /memristor/.bashrc /root/.bashrc

# Copy first-time setup script
RUN mkdir -p /memristor/.setup && \
    chown memristor:memristor /memristor/.setup
COPY ./entrypoint.sh /memristor/.setup/
RUN chmod +x /memristor/.setup/entrypoint.sh
COPY ./Makefile /memristor/.setup/

# Login setup
USER memristor
WORKDIR /memristor

# Container startup
CMD ["/memristor/.setup/entrypoint.sh"]
