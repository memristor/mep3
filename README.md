# mep3
Memristor Eurobot Platform based on ROS 2
![image](https://user-images.githubusercontent.com/13640533/156475608-3f8c7692-c462-4a7d-8078-786c2713d709.png)

## Installation

Make sure you have [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) installed and follow the steps below:

```sh
# Source ROS 2
source /opt/ros/humble/local_setup.bash

# Create a workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone git@github.com:memristor/mep3.git src/mep3
# On embedded device: touch src/mep3/mep3_simulation/COLCON_IGNORE

# Install dependencies
sudo apt install python3-vcstool
# vcs import src < src/mep3/mep3.repos
rosdep update
rosdep install --from-paths src --ignore-src -r

# Build the packages
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source this workspace
source install/local_setup.bash
```

⚠️ **Recommended:**
Please check alternative installation methods [here](./docker).

## Bringup

Start the ROS app on a physical robot or a simulation with multiple robots.

### Physical

Start the ROS app on SBC (we support Raspberry Pi):
```sh
ros2 launch mep3_bringup robot_launch.py bt:=false color:=blue
```

Important parameters are:
- `namespace`: Whether to load a configuration for a big or small robot (can be `big` or `small`).
- `color`: Team color (can be `blue` or `green`).
- `bt`: Whether to run a behavior tree (can be `true` or `false`).
- `strategy`: Name of the behavior tree you want run.

### Simulation

Start the simulation on a PC:
```sh
ros2 launch mep3_bringup simulation_launch.py
```

Important parameters are:
- `color`: Team color (can be `blue` or `green`).
- `bt`: Whether to run a behavior tree (can be `true` or `false`).
- `big_strategy`: Name of the behavior tree you want run for the big robot.
- `small_strategy`: Name of the behavior tree you want run for the small robot.

### Demo

#### Teleoperation
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=big/cmd_vel
```
(use `i`, `j`, `l`, and `,` keys move to the robot)

#### Visualization & navigation
```sh
ros2 launch mep3_bringup rviz_launch.py namespace:=big
```
(use `2D Goal Pose` to navigate the robot)

## Tips

### Live strategies

To iterate quickly on behaviors you can load any strategy that contains `live` in the name.
As soon as any of the files in the [`mep3_behavior/strategies`](mep3_behavior/strategies) directory is changed the behavior node will reload.

### Webots world simulation

- Open [`mep3_simulation/webots_data/worlds/eurobot.wbt`](./mep3_simulation/webots_data/worlds/eurobot.wbt) in Webots
  ```sh
  webots ~/ros2_ws/src/mep3/mep3_simulation/webots_data/worlds/eurobot.wbt
  ```
- Stop simulation and set time to `00:00:00`
- Save changes
- Commit all changes except for [`Viewpoint`](./mep3_simulation/webots_data/worlds/eurobot.wbt#L5-L7)

### Testing

- Change working directory to `~/ros2_ws`
- Run the following command:
  ```sh
  source /opt/ros/humble/local_setup.bash
  colcon test --event-handlers console_cohesion+ --return-code-on-test-failure
  ```
