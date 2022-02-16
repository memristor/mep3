VNC_DISPLAY	!=	echo '${VNC_HOST_PORT}' | awk '{ sub(/590?/, "", $$0); print $$0 }'
NOVNC_PORT	=	680${VNC_DISPLAY}

vnc-setup: turbovnc virtualgl novnc vnc-window-manager vnc-script vnc-entrypoint

turbovnc:
	wget -nv -O ./turbovnc.deb "https://downloads.sourceforge.net/project/turbovnc/2.2.90%20%283.0%20beta1%29/turbovnc_2.2.90_amd64.deb?use_mirror=autoselect"
	sudo -E apt-get install -y ./turbovnc.deb
	rm -f ./turbovnc.deb
	echo 'PATH="$$PATH:/opt/TurboVNC/bin"' >> /memristor/.bashrc

virtualgl:
	wget -nv -O ./virtualgl.deb "https://downloads.sourceforge.net/project/virtualgl/3.0/virtualgl_3.0_amd64.deb?use_mirror=autoselect"
	sudo -E apt-get install -y ./virtualgl.deb
	rm -f ./virtualgl.deb
	echo 'PATH="$$PATH:/opt/VirtualGL/bin"' >> /memristor/.bashrc
	printf "1\n\n\n\nX" | sudo /opt/VirtualGL/bin/vglserver_config
	sudo usermod -a -G vglusers memristor

novnc:
	sudo -E apt-get install -y novnc
	sudo ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

vnc-script:
	echo '#!/bin/sh' > /memristor/.setup/vnc.sh
	echo '/usr/bin/websockify --web /usr/share/novnc/ ${NOVNC_PORT} 127.0.0.1:${VNC_HOST_PORT} &' >> /memristor/.setup/vnc.sh
	echo 'while true; do' >> /memristor/.setup/vnc.sh
	echo 'cp -p /memristor/.Xauthority.host /memristor/.Xauthority' >> /memristor/.setup/vnc.sh
	echo 'sudo rm -f /tmp/.X11-unix/X${VNC_DISPLAY} /tmp/.X${VNC_DISPLAY}-lock' >> /memristor/.setup/vnc.sh
	echo '/opt/TurboVNC/bin/vncserver -securitytypes TLSNone,X509None,None -log /memristor/.vnc/docker.log -wm startxfce4 -vgl -fg :${VNC_DISPLAY}' >> /memristor/.setup/vnc.sh
	echo 'sleep 10' >> /memristor/.setup/vnc.sh
	echo 'done' >> /memristor/.setup/vnc.sh
	chmod +x /memristor/.setup/vnc.sh

vnc-entrypoint:
	sed 's|cd && exec bash||' -i /memristor/.setup/entrypoint.sh
	echo 'exec /memristor/.setup/vnc.sh' >> /memristor/.setup/entrypoint.sh

vnc-window-manager:
	echo '/usr/sbin/lightdm' | sudo tee /etc/X11/default-display-manager
	sudo -E apt-get install -y --no-install-recommends xorg dbus-x11 xfce4 xfce4-terminal firefox
