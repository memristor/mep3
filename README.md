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
# On embedded device: touch src/mep3/mep3_simulation/COLCON_IGNORE

# Install dependencies
sudo apt install python3-vcstool
vcs import src < src/mep3/mep3.repos
rosdep update
rosdep install --from-paths src --ignore-src

# Create udev rules so rplidar and dynamixel usb ports static names
sudo cp src/mep3/tools/rplidar.rules /etc/udev/rules.d/
sudo cp src/mep3/tools/dynamixel.rules /etc/udev/rules.d/

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

### BehaviorTree strategies

Robot strategies are located inside [mep3_behavior_tree/assets](./mep3_behavior_tree/assets)
directory with the following hierarchy:

```ini
mep3_behavior_tree/assets
  - skills/
    - big_retract_hands.xml         # skill for Big robot
    - small_replace_statuette.xml   # skill for Small robot
    - common_retract_hands.xml      # skill for both robots
  - tasks/
    - small_collect_dispenser.xml   # task for Small robot
    - big_fill_work_shed.xml        # task for Big robot
  - strategies/
    - big/
      - purple_strategy.xml         # default strategy for Big robot
      - test_strategy_2.xml         # example test strategy
    - small/
      - purple_strategy.xml         # default strategy for Small robot
```

Example skill:
```xml
<root main_tree_to_execute="skill_retract_hands">
    <BehaviorTree ID="skill_retract_hands">
        <Parallel failure_threshold="1" success_threshold="6">
            <Dynamixel label="arm_right_motor_base" position="0" />
            <Dynamixel label="arm_left_motor_base" position="0" />
        </Parallel>
    </BehaviorTree>
</root>
```

Example task:
```xml
<root main_tree_to_execute="WorkingShed">
    <include path="../../skills/common_scoreboard.xml" />
    <BehaviorTree ID="WorkingShed">
        <SequenceStar>
            <Navigate name="1;1;180" />
            <SubTree ID="ScoreboardWorkShed" __shared_blackboard="true" />
        </SequenceStar>
    </BehaviorTree>
</root>
```

Example strategy:
```xml
<root main_tree_to_execute="BehaviorTree">
    <include path="../tasks/small_working_shed.xml" />
    <BehaviorTree ID="BehaviorTree">
        <SequenceStar>
            <Wait duration="5.0" name="Wait a bit" />
            <Navigate name="1;1;180" />
            <SubTree ID="WorkingShed" __shared_blackboard="true" />
        </SequenceStar>
    </BehaviorTree>
</root>
```

Strategy file should include tasks, which in turn include skills.
Skill subtrees can also be called from strategies directly.

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
