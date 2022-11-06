#!/bin/sh

export VGL_DISPLAY=${DISPLAY}

touch /tmp/vnc.pid
old_pid="$(cat /tmp/vnc.pid)"

if kill -0 $old_pid; then
    exit 1
else
    echo $$ > /tmp/vnc.pid
fi

/usr/bin/websockify --web /usr/share/novnc/ 6810 127.0.0.1:5910 &

while true; do
    cp -p /memristor/.host/.Xauthority /memristor/.Xauthority
    sudo rm -f /tmp/.X11-unix/X10 /tmp/.X10-lock
    /opt/TurboVNC/bin/vncserver \
        -securitytypes TLSNone,X509None,None \
        -log /memristor/.vnc/docker.log \
        -wm xfce \
        ${VGL} \
        -fg \
        :10
done
