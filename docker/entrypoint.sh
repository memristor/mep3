#!/bin/sh

export DEBIAN_FRONTEND=noninteractive
export DEBCONF_NONINTERACTIVE_SEEN=true

! [ -f /memristor/.setup/completed ] && \
cd /memristor/.setup && exec make "provision-$PLATFORM"
cd && exec bash
