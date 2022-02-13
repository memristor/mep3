vnc-setup: turbovnc virtualgl vnc-passwd vnc-entrypoint vnc-wm
	
turbovnc:
	wget -nv -O ./turbovnc.deb https://deac-ams.dl.sourceforge.net/project/turbovnc/2.2.90%20%283.0%20beta1%29/turbovnc_2.2.90_amd64.deb
	sudo apt-get install -y ./turbovnc.deb
	rm -f ./turbovnc.deb
	echo 'PATH="$$PATH:/opt/TurboVNC/bin"' >> /memristor/.bashrc

virtualgl:
	wget -nv -O ./virtualgl.deb https://altushost-swe.dl.sourceforge.net/project/virtualgl/3.0/virtualgl_3.0_amd64.deb
	sudo apt-get install -y ./virtualgl.deb
	rm -f ./virtualgl.deb
	echo 'PATH="$$PATH:/opt/VirtualGL/bin"' >> /memristor/.bashrc
	printf "1\n\n\n\nX" | sudo /opt/VirtualGL/bin/vglserver_config

vnc-passwd:
	printf "robotics\nrobotics\n\n" | sudo passwd memristor

vnc-entrypoint:
	sed 's|cd && bash||' -i /memristor/.setup/entrypoint.sh
	echo 'vncserver -securitytypes plain -log /memristor/.vnc/docker.log -wm startxfce4 :1 &' >> /memristor/.setup/entrypoint.sh
	echo 'tail -f /memristor/.vnc/docker.log'  >> /memristor/.setup/entrypoint.sh
	echo 'cd && bash'  >> /memristor/.setup/entrypoint.sh

vnc-wm:
	DEBIAN_FRONTEND=noninteractive sudo apt-get install -y xfce4 lightdm
