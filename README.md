# mep3
Memristor Eurobot Platform based on ROS2

## Getting Started


- Install `git` and `git-lfs` (see [tutorial](https://git-lfs.github.com/))
- Create a new workspace `foxy_ws` and source `foxy_ws/src` directory
```sh
mkdir -p foxy_ws/src
```
- Clone this repository to `foxy_ws/src/mep3`
```sh
git clone https://github.com/memristor/mep3.git ./foxy_ws/src/mep3
```

## Editing Webots world

- Install [Webots R2021b](https://github.com/cyberbotics/webots/releases/download/R2021b/webots_2021b_amd64.deb)
- Open [`webots_data/worlds/eurobot_2022.wbt`](./webots_data/worlds/eurobot_2022.wbt) in Webots
- Stop simulation and set time to `00:00:00`
- Save changes
- Commit all changes except for [`Viewpoint`](./webots_data/worlds/eurobot_2022.wbt#L5-L7)

## ROS2 setup

- Install [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
- Change working directory to `foxy_ws`
- Configure ROS2 workspace
```sh
source /opt/ros/foxy/local_setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro  foxy
```

## Running the simulation

- Change working directory to `foxy_ws`
- Build files (and rebuild on every modification):
```sh
colcon build
source ./install/local_setup.bash
```
- Run the simulation
```sh
ros2 launch mep3_simulation robot_launch.py
```
- Control the robot from another terminal window
```sh
# Note: You need to be in the `foxy_ws` directory 
#       and run commands above starting with `source`

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```