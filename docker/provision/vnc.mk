VNC_DISPLAY	!=	echo '${VNC_HOST_PORT}' | awk '{ sub(/590?/, "", $$0); print $$0 }'

vnc-setup: turbovnc virtualgl vnc-window-manager vnc-script vnc-entrypoint

turbovnc:
	wget -nv -O ./turbovnc.deb "https://downloads.sourceforge.net/project/turbovnc/2.2.90%20%283.0%20beta1%29/turbovnc_2.2.90_amd64.deb?ts=gAAAAABiCTM5OO6vxolPny2srVISQf6mZ-9O1vTxAGZsOCferRYGKwVMmrd8fY7t5QV787Hzmrq5DXn9Xu4kHzC9VyyEgRtJ0g%3D%3D&use_mirror=autoselect&r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fturbovnc%2Ffiles%2F2.2.90%2520%25283.0%2520beta1%2529%2Fturbovnc_2.2.90_amd64.deb%2Fdownload"
	sudo -E apt-get install -y ./turbovnc.deb
	rm -f ./turbovnc.deb
	echo 'PATH="$$PATH:/opt/TurboVNC/bin"' >> /memristor/.bashrc

virtualgl:
	wget -nv -O ./virtualgl.deb "https://downloads.sourceforge.net/project/virtualgl/3.0/virtualgl_3.0_amd64.deb?ts=gAAAAABiCTO32yZa5GkYAP2YObp1Ts6UgHCB2-dgX3FSkvMfj_dydM-Vj7kqkGtfklQdKyGH-x4WdtXrjvxkIDb8riiHhrvBUg%3D%3D&use_mirror=autoselect&r=https%3A%2F%2Fsourceforge.net%2Fprojects%2Fvirtualgl%2Ffiles%2F3.0%2Fvirtualgl_3.0_amd64.deb%2Fdownload"
	sudo -E apt-get install -y ./virtualgl.deb
	rm -f ./virtualgl.deb
	echo 'PATH="$$PATH:/opt/VirtualGL/bin"' >> /memristor/.bashrc
	printf "1\n\n\n\nX" | sudo /opt/VirtualGL/bin/vglserver_config
	sudo usermod -a -G vglusers memristor

vnc-script:
	echo '#!/bin/sh' > /memristor/.setup/vnc.sh
	echo 'while true; do' >> /memristor/.setup/vnc.sh
	echo 'sudo rm -f /tmp/.X11-unix/X${VNC_DISPLAY} /tmp/.X${VNC_DISPLAY}-lock' >> /memristor/.setup/vnc.sh
	echo '/opt/TurboVNC/bin/vncserver -securitytypes TLSNone,X509None,None -log /memristor/.vnc/docker.log -wm startxfce4 -vgl -fg :${VNC_DISPLAY}' >> /memristor/.setup/vnc.sh
	echo 'sleep 60' >> /memristor/.setup/vnc.sh
	echo 'done' >> /memristor/.setup/vnc.sh
	chmod +x /memristor/.setup/vnc.sh

vnc-entrypoint:
	sed 's|cd && exec bash||' -i /memristor/.setup/entrypoint.sh
	echo 'exec /memristor/.setup/vnc.sh' >> /memristor/.setup/entrypoint.sh

vnc-window-manager:
	echo '/usr/sbin/lightdm' | sudo tee /etc/X11/default-display-manager
	sudo -E apt-get install -y --no-install-recommends xorg dbus-x11 xfce4 xfce4-terminal firefox
 