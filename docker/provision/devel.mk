webots:
	wget -nv -O ./webots.deb 'https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb'
	sudo -E apt-get install -y ./webots.deb
	rm -f ./webots.deb
	test -d /memristor/.config/Cyberbotics || mkdir -p /memristor/.config/Cyberbotics && rsync -rP /config/Cyberobotics/ /memristor/.config/Cyberbotics/

groot:
	wget -nv -O ./Groot.AppImage 'https://github.com/BehaviorTree/Groot/releases/download/1.0.0/Groot-1.0.0-x86_64.AppImage'
	chmod +x ./Groot.AppImage
	./Groot.AppImage --appimage-extract
	rm -f ./Groot.AppImage
	sudo mv ./squashfs-root /opt/groot
	find /opt/groot -type d -exec sudo chmod 755 {} \;
	sudo ln -sf /opt/groot/AppRun /usr/bin/groot
	sudo chmod +x /usr/bin/groot
	sudo -E apt-get install -y mesa-utils libharfbuzz0b libgtk-3-dev

vscode:
	wget -nv -O ./vscode.deb 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64'
	sudo -E apt-get install -y ./vscode.deb
	rm -f ./vscode.deb
	sudo -E apt-get install -y alsa libxshmfence1 libgtk-3-dev
	test -d /memristor/ros2_ws/.vscode || rsync -rP ./config/.vscode/ /memristor/ros2_ws/.vscode/
	code --install-extension CoenraadS.bracket-pair-colorizer-2
	code --install-extension eamodio.gitlens
	code --install-extension ms-python.python
	code --install-extension ms-vscode.cpptools-extension-pack
	code --install-extension usernamehw.errorlens
