# mep3
Memristor Eurobot Platform based on ROS 2

## Table of contents
- [mep3](#mep3)
  - [Table of contents](#table-of-contents)
  - [Getting started](#getting-started)
    - [Local development environment](#local-development-environment)
    - [Remote development environment (VNC)](#remote-development-environment-vnc)
    - [Manual installation](#manual-installation)
  - [Webots world simulation](#webots-world-simulation)
  - [ROS 2 platform](#ros-2-platform)
    - [Compilaton](#compilaton)
    - [Running the simulation](#running-the-simulation)
    - [Navigation 2 stack](#navigation-2-stack)
    - [Testing](#testing)
    - [BehaviorTree strategy planning](#behaviortree-strategy-planning)

## Getting started

### Local development environment

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

### Remote development environment (VNC)

1) Follow [Local development environment](#local-development-environment) steps 1 to 4
1) Run provisioning script
    ```sh
    cd ~/ros2_ws/src/mep3/docker
    make VNC_HOST_DISPLAY=$DISPLAY VNC_HOST_PORT=5902 vnc
    ```
    Set `VNC_HOST_PORT` to an unoccupied port, preferably around *5900*

    Web-based VNC client will be accessible at `http://localhost:PORT/`, where `PORT = VNC_HOST_PORT + 900`.

### Manual installation

1) Follow [Local development environment](#local-development-environment) steps 1 to 4
1) Install `ros-galactic`, `webots`, `groot` and other dependencies
    ```sh
    # Assuming Ubuntu 20.04 LTS
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
1) Initialize and update rosdep
    ``` sh
    cd ~/ros2_ws
    sudo rosdep init
    rosdep --rosdistro galactic update
    rosdep --rosdistro galactic install --from-paths src --ignore-src
    ```
## Webots world simulation

- Open [`mep3_simulation/webots_data/worlds/eurobot_2022.wbt`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt) in Webots
```sh
nohup webots ~/ros2_ws/src/mep3/mep3_simulation/webots_data/worlds/eurobot_2022.wbt &
```
- Stop simulation and set time to `00:00:00`
- Save changes
- Commit all changes except for [`Viewpoint`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt#L5-L7)
## ROS 2 platform

### Compilaton

- Change working directory to `~/ros2_ws`
- Install dependencies if there are changes in `package.xml` files 
```sh
rosdep install --from-paths src --ignore-src
```
- Build files (and rebuild on every modification):
```sh
colcon build
source ./install/local_setup.bash
```

### Running the simulation
- Run the simulation
```sh
ros2 launch mep3_bringup simulation_launch.py
```
- Control the robot from another terminal window
```sh
source /opt/ros/galactic/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=big/cmd_vel
```

### Navigation 2 stack

To launch simulation with `nav2` run:
```sh
ros2 launch mep3_bringup simulation_launch.py
```

Open RViz afterwards using:
```sh
ros2 launch mep3_bringup rviz_launch.py
```

### Testing

- Change working directory to `~/ros2_ws`
- Run the following command:
```sh
source /opt/ros/galactic/local_setup.bash
colcon test --event-handlers console_cohesion+ --return-code-on-test-failure
```

### BehaviorTree strategy planning

- Run Groot
```sh
nohup groot &
```
- Edit strategies XML files in [mep3_behavior_tree/assets/strategies](./mep3_behavior_tree/assets/strategies) directory
- Run planner for `ros_demo.xml` with:
```sh
ros2 run mep3_behavior_tree mep3_behavior_tree ros_demo
```
