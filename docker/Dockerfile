FROM ubuntu:20.04

RUN apt-get update

RUN echo 'Europe/Belgrade' > /etc/timezone && \
    echo 'LANG=en_US.UTF-8' > /etc/locale.conf && \
    ln -sf /usr/share/zoneinfo/Europe/Belgrade /etc/localtime && \
    apt-get install -y --no-install-recommends tzdata locales && \
    locale-gen

RUN apt-get -y install git git-lfs wget curl sudo python3 python3-pip

RUN useradd -d /mep3 -m \
            -u 1000 -U \
            -s /usr/bin/bash \
            -c "Memristor Robotics" mep3 && \
    echo 'mep3 ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

USER mep3
WORKDIR /mep3

ENTRYPOINT bash
