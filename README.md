# mep3
Memristor Eurobot Platform based on ROS 2
![image](https://user-images.githubusercontent.com/13640533/156475608-3f8c7692-c462-4a7d-8078-786c2713d709.png)

## Table of contents
- [mep3](#mep3)
  - [Table of contents](#table-of-contents)
  - [Getting started](#getting-started)
  - [Webots world simulation](#webots-world-simulation)
  - [ROS 2 platform](#ros-2-platform)
    - [Compilaton](#compilaton)
    - [Running the simulation](#running-the-simulation)
    - [Navigation 2 stack](#navigation-2-stack)
    - [Testing](#testing)
    - [BehaviorTree](#behaviortree)
    - [Terminal shortcuts](#terminal-shortcuts)

## Getting started

```sh
# Make sure you have ROS 2 Galactic installed.
# https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html

# Source ROS 2
source /opt/ros/galactic/local_setup.bash

# Create a workspace
mkdir -p ~/galactic_ws/src
cd ~/galactic_ws
git clone git@github.com:memristor/mep3.git src/mep3

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src

# Build the packages
colcon build

# Source this workspace
source install/local_setup.bash
```

Please check alternative installation methods [here](./docker).

## Webots world simulation

- Open [`mep3_simulation/webots_data/worlds/eurobot_2022.wbt`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt) in Webots
```sh
webots ~/ros2_ws/src/mep3/mep3_simulation/webots_data/worlds/eurobot_2022.wbt
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
- Run the simulation without the behavior tree:
```sh
ros2 launch mep3_bringup simulation_launch.py bt:=false
```
- Control the robot from another terminal window
```sh
source /opt/ros/galactic/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=big/cmd_vel
```
- Run the simulation with the behavior tree:
```sh
ros2 launch mep3_bringup simulation_launch.py
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

### BehaviorTree

To edit strategies you can use [Groot](https://github.com/BehaviorTree/Groot):
- Install Groot (you can use [the AppImage version](https://github.com/BehaviorTree/Groot/releases))
- Edit strategies XML files in [mep3_behavior_tree/assets/strategies](./mep3_behavior_tree/assets/strategies) directory
- Run planner for `first_strategy.xml` with:
  ```sh
  ros2 run mep3_behavior_tree mep3_behavior_tree first_strategy --ros-args -r __ns:=/big
  ```


### Terminal shortcuts

We use custom terminal shortcuts to provide better development environment ergonomics.

```sh
# Enable terminal shortcuts temporarily
source ~/ros2_ws/src/mep3/docker/config/shortcuts.sh

# Enable terminal shortcuts permanently
echo "source ~/ros2_ws/src/mep3/docker/config/shortcuts.sh" >> ~/.bashrc
source ~/.bashrc
```

After enabling shortcuts, run command `h` in the terminal to get familiarized with them.