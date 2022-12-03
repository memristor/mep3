#!/bin/sh

export VGL_DISPLAY="${DISPLAY}"

PORT_VNC="${PORT_VNC:-5910}"
PORT_WEB="${PORT_WEB:-6810}"
RUN_DISPLAY="$(echo "${PORT_VNC}" | grep -o '[1-9].$\|.$')"

touch /tmp/vnc.pid
old_pid="$(cat /tmp/vnc.pid)"

if kill -0 "$old_pid"; then
    exit
else
    echo $$ > /tmp/vnc.pid
fi

/usr/bin/websockify --web /usr/share/novnc/ "${PORT_WEB}" "127.0.0.1:${PORT_VNC}" &

while true; do
    cp -p /memristor/.host/.Xauthority /memristor/.Xauthority
    sudo rm -f /tmp/.X11-unix/X10 /tmp/.X10-lock
    /opt/TurboVNC/bin/vncserver \
        -securitytypes TLSNone,X509None,None \
        -log /memristor/.vnc/docker.log \
        -wm xfce \
        "${VGL}" \
        -fg \
        ":${RUN_DISPLAY}"
done
