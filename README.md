# mep3
Memristor Eurobot Platform based on ROS 2

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
- Open [`mep3_simulation/webots_data/worlds/eurobot_2022.wbt`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt) in Webots
- Stop simulation and set time to `00:00:00`
- Save changes
- Commit all changes except for [`Viewpoint`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt#L5-L7)

## ROS 2 setup

- Install [ROS 2 foxy](https://docs.ros.org/en/foxy/Installation.html)
- Change working directory to `foxy_ws`
- Configure ROS 2 workspace
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
ros2 launch mep3_simulation simulation_launch.py
```
- Control the robot from another terminal window
```sh
source /opt/ros/foxy/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Navigation

To launch simulation with `rviz` and `nav2` run
```sh
ros2 launch mep3_robot robot_launch.py rviz:=true nav:=true simulation:=true
```

## Testing

- Change working directory to `foxy_ws`
- Run the following command:
```sh
source /opt/ros/foxy/local_setup.bash
colcon test --event-handlers console_cohesion+ --return-code-on-test-failure
```

## BehaviorTree Groot setup

```bash
wget https://github.com/BehaviorTree/Groot/releases/download/1.0.0/Groot-1.0.0-x86_64.AppImage -o Groot.AppImage
chmod +x Groot.AppImage
./Groot.AppImage
```
