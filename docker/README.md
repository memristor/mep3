# Alternative installation methods

## Local development environment

1) Install `git`, `make` and `docker`
1) Run docker daemon and add yourself to docker group
    ```sh
    sudo systemctl enable docker.service
    sudo systemctl start docker.service
    sudo usermod -aG docker $USER
    ```
1) Create a new workspace `ros2_ws` and source `ros2_ws/src` directory
    ```sh
    mkdir -p ~/ros2_ws/src
    ```
1) Clone this repository to `ros2_ws/src/mep3`
    ```sh
    # Make sure to have your SSH keys added to GitHub
    git clone git@github.com:memristor/mep3.git ~/ros2_ws/src/mep3
    ```
1) Run provisioning script 
   ```sh
   cd ~/ros2_ws/src/mep3/docker
   make devel
   ```
1) Wait for the provisioning script to finish
1) Acces the environment from any terminal window
    ```sh
    docker exec -it mep3-devel bash
    ```
    Graphical applications started inside this terminal will use your existing Xorg session to display.

## Remote development environment (VNC)

1) Follow [Local development environment](#local-development-environment) steps 1 to 4
1) Run provisioning script
    ```sh
    cd ~/ros2_ws/src/mep3/docker
    make VNC_HOST_DISPLAY=$DISPLAY VNC_HOST_PORT=5902 vnc
    ```
    Set `VNC_HOST_PORT` to an unoccupied port, preferably around *5900*

    Web-based VNC client will be accessible at `http://localhost:PORT/`, where `PORT = VNC_HOST_PORT + 900`.

    If you are setting up through SSH, make sure to have a running Xorg server on host machine, and set `VNC_HOST_PORT` to its display value (eg `:0`).

## Manual installation on Ubuntu 20.04

1) Follow [Local development environment](#local-development-environment) steps 1 to 4
1) Install `ros-galactic`, `webots`, `groot` and other dependencies
    ```sh
    cd ~/ros2_ws/src/mep3/docker
    make ros-apt ros-desktop
    source /opt/ros/galactic/local_setup.bash
    
    cd ~/ros2_ws
    wget -nv -O ./Groot.AppImage 'https://github.com/BehaviorTree/Groot/releases/download/1.0.0/Groot-1.0.0-x86_64.AppImage'
	chmod +x ./Groot.AppImage
    
    wget -nv -O ./webots.deb 'https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb'
	sudo -E apt-get install -y ./webots.deb
	rm -f ./webots.deb

    mkdir -p ~/.config/Cyberbotics && cp ./src/mep3/docker/config/Cyberobotics/ ~/.config/Cyberbotics/Webots-R2022a.conf
    ```
2) Initialize and update rosdep
    ``` sh
    cd ~/ros2_ws
    sudo rosdep init
    rosdep --rosdistro galactic update
    rosdep --rosdistro galactic install --from-paths src --ignore-src
    ```
