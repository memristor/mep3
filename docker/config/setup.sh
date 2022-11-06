#!/bin/sh

if dialog --title 'mep3 config' --yesno 'Enable VNC' 5 30; then
    VNC_CMD='/usr/bin/vnc.sh &'
    if dialog --title 'mep3 config' --yesno 'Enable GPU acceleration' 5 30; then
       VNC_CMD="VGL=-vgl ${VNC_CMD}"
    fi
    echo "$VNC_CMD # Setup_VNC" >> /memristor/.bashrc
else
    sed '/# Setup_VNC/d' -i /memristor/.bashrc
fi

clear
