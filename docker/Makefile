.PHONY: devel

VNC_HOST_DISPLAY!=	echo $$DISPLAY
VNC_HOST_PORT	?=	5902
UBUNTU_CODENAME	!=	. /etc/os-release && echo $$UBUNTU_CODENAME
ROS_DISTRO	=	galactic
SRC		!=	realpath ./../../../
UID		!=	id -u

include provision/devel.mk
include provision/docker.mk
include provision/mep3.mk
include provision/ros2.mk
include provision/vnc.mk

prod: docker-prepare-prod docker-run-prod docker-logs-prod
devel: docker-prepare-devel docker-run-devel docker-logs-devel
vnc: docker-prepare-vnc-${VNC_HOST_PORT} docker-run-vnc docker-logs-vnc-${VNC_HOST_PORT} 

provision-prod: apt-cache ros-apt ros-base mep3-prod-checkout mep3-rosdep mep3-bashrc done
provision-devel: apt-cache ros-apt ros-desktop mep3-rosdep mep3-bashrc webots groot vscode done
provision-vnc: apt-cache vnc-setup mep3-full-checkout provision-devel

apt-cache:
	sudo -E apt-get update

done:
	touch /memristor/.setup/completed
	@echo "$(tput bold; tput setaf 2)=============================================$(tput sgr0)"
	@echo "$(tput bold; tput setaf 2)Container provisioning completed successfully$(tput sgr0)"
