#!/usr/bin/env bash

_help=false
_configure_proxy=false
_vnc=false
_first_time_ros_setup=false
_enhanced_shell_prompt=false
_shell_shortcuts=false
_all=false
interactive=false

usage() {
	echo "Usage: $0 [--help] [--configure-proxy] [--vnc] [--first-time-ros-setup] [--enhanced-shell-prompt] [--shell-shortcuts] [--interactive]"
	echo "--help print help and exit"
	echo "--interactive do everything interactively"
	echo "--no-default do not use default setup configurations"
	echo "--configure-proxy configures proxy for use in ftn network"
	echo "--vnc configures VNC server in the container for outside remote DE use"
	echo "--first-time-ros-setup configures ros for the first time setup"
	echo "--enhanced-shell-prompt enables starship enhanced shell prompt"
	echo "--shell-shortcuts enables shell shortcuts for easier tools use"
	exit
}

default_configure_proxy() {
	sed '/# Setup_proxy/d' -i /memristor/.bashrc
	sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
}

configure_proxy() {
	if $interactive; then
		# FTN proxy is actually enabled here
		if dialog --title 'mep3 config' --defaultno --yesno 'Enable UNS proxy' 5 30; then
			sed '/# Setup_proxy/d' -i /memristor/.bashrc
			sudo sed '/# Setup_proxy/d' -i /etc/apt/apt.conf
			echo "source /memristor/ros2_ws/src/mep3/docker/config/proxy.sh &>/dev/null # Setup_proxy" >>/memristor/.bashrc
			source /memristor/ros2_ws/src/mep3/docker/config/proxy.sh
			echo 'Acquire::http::Proxy "http://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
			echo 'Acquire::https::Proxy "https://ftn.proxy:8080"; # Setup_proxy' | sudo tee -a /etc/apt/apt.conf
			return
		fi
	fi
	default_configure_proxy
}

default_first_time_ros_setup() {
	clear
	sudo -E rosdep init
	sudo -E apt-get install -y python3-vcstool
	rosdep --rosdistro "${ROS_DISTRO}" update
	cd /memristor/ros2_ws && yes | rosdep --rosdistro "${ROS_DISTRO}" install -r --from-paths src --ignore-src
}

first_time_ros_setup() {
	if $interactive; then
		if dialog --title 'mep3 config' --yesno 'Run first time ROS setup' 5 30; then
			default_first_time_ros_setup
			return
		fi
	fi
	default_first_time_ros_setup
}

default_enhanced_shell_prompt() {
	sed '/# Setup_prompt/d' -i /memristor/.config/fish/config.fish
	echo 'starship init fish | source # Setup_prompt' >>/memristor/.config/fish/config.fish
	eval "curl -sS https://starship.rs/install.sh | sh -s -- --yes"
}

enhanced_shell_prompt() {
	if $interactive; then
		if dialog --title 'mep3 config' --yesno 'Enable enhanced shell prompt' 5 38; then
			default_enhanced_shell_prompt
			return
		else
			sed '/# Setup_prompt/d' -i /memristor/.config/fish/config.fish
		fi
	fi
	default_enhanced_shell_prompt
}

default_shell_shortcuts() {
	sed '/# Setup_shortcuts/d' -i /memristor/.config/fish/config.fish
	echo "source /memristor/ros2_ws/src/mep3/docker/config/fish/shortcuts.fish &>/dev/null # Setup_shortcuts" >>/memristor/.config/fish/config.fish
	echo "source /memristor/ros2_ws/src/mep3/docker/config/fish/git.fish &>/dev/null # Setup_shortcuts" >>/memristor/.config/fish/config.fish
}

shell_shortcuts() {
	if $interactive; then
		if dialog --title 'mep3 config' --yesno 'Enable shell shortcuts' 5 30; then
			default_shell_shortcuts
			return
		else
			sed '/# Setup_shortcuts/d' -i /memristor/.config/fish/config.fish
		fi
	fi
	default_shell_shortcuts
}

interactive_vnc() {
	if dialog --title 'mep3 config' --yesno 'Enable VNC' 5 30; then
		VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
		if dialog --title 'mep3 config' --yesno 'Enable GPU acceleration' 5 30; then
			VNC_CMD="VGL=-vgl ${VNC_CMD}"
		fi
		if dialog --title 'mep3 config' --inputbox 'Enter host X11 display server location' 8 30 "$DISPLAY" 2>/tmp/answer; then
			VNC_CMD=" DISPLAY=$(cat /tmp/answer) ${VNC_CMD}"
		fi
		if dialog --title 'mep3 config' --inputbox 'Enter VNC server port' 8 30 5910 2>/tmp/answer; then
			VNC_CMD=" PORT_VNC=$(cat /tmp/answer) ${VNC_CMD}"
		fi
		if dialog --title 'mep3 config' --inputbox 'Enter noVNC webserver port' 8 30 6810 2>/tmp/answer; then
			VNC_CMD=" PORT_WEB=$(cat /tmp/answer) ${VNC_CMD}"
		fi
		sed '/# Setup_VNC/d' -i /memristor/.bashrc
		echo "$VNC_CMD # Setup_VNC" >>/memristor/.bashrc
	else
		sed '/# Setup_VNC/d' -i /memristor/.bashrc
	fi
}

default_vnc() {
	VNC_CMD='/usr/bin/vnc.sh &>/dev/null'
	VNC_CMD="VGL=-vgl DISPLAY=$DISPLAY PORT_VNC=5910 PORT_WEB=6810 ${VNC_CMD}"
	sed '/# Setup_VNC/d' -i /memristor/.bashrc
	echo "$VNC_CMD # Setup_VNC" >>/memristor/.bashrc
}

vnc() {
	if test -f /opt/TurboVNC/bin/vncserver; then
		echo "TurboVNC is not installed"
		exit 1
	fi
	if $interactive; then
		interactive_vnc
		return
	fi
	default_vnc
}

finalize() {
	sed '/# Setup_shell/d' -i /memristor/.bashrc
	echo 'echo "$-" | grep i -q && exec fish # Setup_shell' >>/memristor/.bashrc

	clear
}

while [ "$#" -gt 0 ]; do
	case "$1" in
	--configure-proxy)
		_configure_proxy=true
		;;
	--vnc)
		_vnc=true
		;;
	--first-time-ros-setup)
		_first_time_ros_setup=true
		;;
	--enhanced-shell-prompt)
		_enhanced_shell_prompt=true
		;;
	--shell-shortcuts)
		_shell_shortcuts=true
		;;
	--interactive)
		interactive=true
		;;
	--help)
		_help=true
		;;
	--all)
		_all=true
		;;
	*)
		usage
		;;
	esac
	shift
done

if $_help; then
	usage
fi

if $_all; then
	configure_proxy
	first_time_ros_setup
	enhanced_shell_prompt
	shell_shortcuts
	vnc
else
	if $_configure_proxy; then
		configure_proxy
	fi
	if $_first_time_ros_setup; then
		first_time_ros_setup
	fi
	if $_enhanced_shell_prompt; then
		enhanced_shell_prompt
	fi
	if $_shell_shortcuts; then
		shell_shortcuts
	fi
	if $_vnc; then
		vnc
	fi
fi
finalize
