# mep3
Memristor Eurobot Platform based on ROS 2

## Getting started

- Install `git`
- Create a new workspace directory `ros2_ws` and source `ros2_ws/src` directory
```sh
mkdir -p ros2_ws/src
```
- Clone this repository to `ros2_ws/src/mep3`
```sh
git clone https://github.com/memristor/mep3.git ./ros2_ws/src/mep3
```
- [Set up `docker`](https://docs.docker.com/engine/install/) on your system and
[add yourself to `docker` group](https://www.configserverfirewall.com/ubuntu-linux/add-user-to-docker-group-ubuntu/)
- Create development environment container
```sh
cd ./ros2_ws/src/mep3/docker
make devel
```
- Wait for the provisioning script to finish
- Open a new terminal and type the following to access the container
```sh
# When inside ./ros2_ws/src/mep3/docker
make bash-devel

# Anywhere on host system
docker exec -it mep3-devel bash
```
**All commands in the remainder of this document will assume that you are inside the container**

## Editing Webots world

- Open [`mep3_simulation/webots_data/worlds/eurobot_2022.wbt`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt) in Webots
```sh
nohup webots ~/ros2_ws/src/mep3/mep3_simulation/webots_data/worlds/eurobot_2022.wbt &
```
- Stop simulation and set time to `00:00:00`
- Save changes
- Commit all changes except for [`Viewpoint`](./mep3_simulation/webots_data/worlds/eurobot_2022.wbt#L5-L7)
## Running the simulation

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
- Run the simulation
```sh
ros2 launch mep3_bringup simulation_launch.py
```
- Control the robot from another terminal window
```sh
source /opt/ros/galactic/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=big/cmd_vel
```

### Navigation

To launch simulation with `nav2` run:
```sh
ros2 launch mep3_bringup simulation_launch.py
```

Open RViz afterwards using:
```sh
ros2 launch mep3_bringup rviz_launch.py
```

## Testing

- Change working directory to `~/ros2_ws`
- Run the following command:
```sh
source /opt/ros/galactic/local_setup.bash
colcon test --event-handlers console_cohesion+ --return-code-on-test-failure
```

## Planning — BehaviorTree strategies

- Run Groot
```sh
nohup groot &
```
- Edit strategies XML files in [mep3_behavior_tree/assets/strategies](./mep3_behavior_tree/assets/strategies) directory
- Run planner for `ros_demo.xml` with:
```sh
ros2 run mep3_behavior_tree mep3_behavior_tree ros_demo
```
