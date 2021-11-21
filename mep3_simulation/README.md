# Webots + ROS2 simulation

## Editing Webots world

- Open [`webots_data/worlds/eurobot_2022.wbt`](./webots_data/worlds/eurobot_2022.wbt) in Webots
- Stop simulation and set time to `00:00:00`
- Save changes
- Commit all changes except for [`Viewpoint`](./webots_data/worlds/eurobot_2022.wbt#L5-L7)

## ROS2 setup

- Install [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)
- Create a new directory (eg. `foxy_ws`)
- Move repository into `foxy_ws/src` so its root directory is `foxy_ws/src/mep3`
- Change working directory to `foxy_ws`
- Set up the [`webots_ros2`](https://github.com/cyberbotics/webots_ros2/wiki/)

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