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
RUN apt-get -y install apt-utils gnupg2 lsb-release sudo rsync python3 python3-pip \
               git git-lfs wget curl vim pciutils keyboard-configuration

COPY ./config/ps1.sh /tmp

# User setup
RUN useradd -d /memristor -m \
            -u $user_id -U \
            -s /usr/bin/bash \
            -c "Memristor Robotics" memristor && \
    echo 'memristor ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers && \
    cat /tmp/ps1.sh | tee -a /memristor/.bashrc /root/.bashrc && \
    echo 'source /memristor/.setup/config/shortcuts.sh' | tee -a /memristor/.bashrc

# Copy first-time setup script and user configuration
RUN mkdir -p /memristor/.setup
COPY ./entrypoint.sh /memristor/.setup/
RUN chmod +x /memristor/.setup/entrypoint.sh
COPY ./config /memristor/.setup/config
COPY ./Makefile /memristor/.setup/
COPY ./provision /memristor/.setup/provision
RUN chown -R $user_id:$user_id /memristor/.setup

# Login setup
USER memristor
WORKDIR /memristor

# Container startup
CMD ["/memristor/.setup/entrypoint.sh"]
