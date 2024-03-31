# Move

> Based on [`mep3_navigation::MoveBehavior`](https://github.com/memristor/mep3/edit/main/mep3_navigation/src/move_behavior/README.md).

Local navigation. Low-level mobile robot motion package for short distances (e.g. up to 1m). Designed for applications such as visual positioning, docking, inplace rotation, and odometry calibration.

Main features:
- **Accurate position control.** Implements the distance-angle controller.
- **Motion generation.** Limits velocity, acceleration, and jerk.
- **Servoing.** Pose tracking, accepts continuous position commands.
- **Obstacle detection.** Integrates Nav2 costmaps to detect obstacles on a simulated path.
- **Frame transformations.** Receives (x, y, yaw) in a target frame and regulates position in the `odom` frame.
- **Latency compensation.** Utilizes odometry + TF buffer to compensate for the latency. 
- **Debouncing.** Regulates position until the robot completely stops.
- **Stuck detection (WIP).** Implements a robust stuck detection algorithm.

## Interface

| Name | Direction | Type | Description |
|---|---|---|---|
| `cmd_vel`   | publishes | [`geometry_msgs/msg/Twist`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) | Robot's velocity command |
| `~/state`   | publishes | [`spes_msgs/msg/MoveState`](../spes_msgs/msg/MoveState.msg) | Move state details |
| `~/command` | subscribes | [`spes_msgs/msg/MoveState`](../spes_msgs/msg/MoveCommand.msg) | Moves a robot to the target pose, can be continuous |
| `/tf`       | subscribes | [`tf2_msgs/msg/TFMessage`](http://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html) | Uses the TF tree to resolve odom, global, target, and base frames |
| `local_costmap/costmap_raw`  | subscribes (optional) | [`nav2_msgs/msg/Costmap`](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/msg/Costmap.msg) | Costmap for obstacles avoidance  |
| `local_costmap/published_footprint` | subscribes (optional) | [`geometry_msgs/msg/PolygonStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PolygonStamped.html) | Robot footprint |
| `~/move`    | action server | [`spes_msgs/action/Move`](../spes_msgs/action/Move.action) | Moves a robot to the target pose |

## Frames

There are four important frames:
- We defined an objective as a `target` frame (2D pose) in the `global` frame.
- The robot (`base` frame) finds the target in the `odom` frame and continues motion in the `odom` frame.

Therefore, there are two stages:
- Once the command is received we calculate the `target` pose in the `odom` frame: $$ T_{odom}^{target} = (T_{global}^{odom})^{-1} * T_{global}^{target} $$
- In the control loop we regulate the position according to the error: $$ T_{base}^{target} = (T_{odom}^{base})^{-1} * T_{odom}^{target} $$

In the typical example `global` = `map` and `odom` = `odom`.
It means the robot moves to the absolute pose, but it uses the `odom` frame to avoid discrete pose jumps.

However, `global` + `odom` frames give us the flexibility to achieve useful behaviors:
- If `global` = `base_link` and `odom` = `odom` then the robot moves relative to its current pose.
- If `global` = `marker_1` (a fiducial marker) and `odom` = `marker_1` then the robot moves to the marker pose (e.g. to pick the object). 
- However, if the robot loses the `marker_1` from its sight then we can set the following `global` = `marker_1` and `odom` = `odom`.

## Configuration

Find a configuration example below:
```yaml
move:
    ros__parameters:
        update_rate: 50
        command_timeout: 0.5

        # Can be overridden by the command.
        linear:
            kp: 0.5
            kd: 0.0
            max_velocity: 0.5
            max_acceleration: 0.5
            tolerance: 0.1
        angular:
            kp: 0.5
            kd: 0.0
            max_velocity: 0.5
            max_acceleration: 0.5
            tolerance: 0.1
```

## Command Examples

Some ideas on how to utilize the move behavior.

> Make sure to the `move` as `ros2 run spes_move move` before executing the examples.`;

Move 20cm forward and stop:
```bash
ros2 topic pub -1 move/command spes_msgs/msg/MoveCommand '{ "header": { "frame_id": "base_link" }, "odom_frame": "odom", "target": { "x": 0.2 }, "mode": 2 }'
```

Keep moving forward until canceled:
```bash
ros2 topic pub -r1 move/command spes_msgs/msg/MoveCommand '{ "header": { "frame_id": "base_link" }, "odom_frame": "odom", "target": { "x": 0.5 }, "mode": 2 }'
```

Rotate in place for 90 degrees:
```bash
ros2 topic pub -1 move/command spes_msgs/msg/MoveCommand '{ "header": { "frame_id": "base_link" }, "odom_frame": "odom", "target": { "theta": 1.507 }, "mode": 1 }'
```

Move to pose (-0.5, -0.5):
```bash
ros2 topic pub -1 move/command spes_msgs/msg/MoveCommand '{ "header": {"frame_id": "odom" }, "odom_frame": "odom", "target": { "x": -0.5, "y": -0.5 } }'
```

Move to pose (-0.5, -0.5) using action:
```bash
ros2 action send_goal move/move spes_msgs/action/Move '{ "header": {"frame_id": "odom" }, "odom_frame": "odom", "target": { "x": -0.5, "y": -0.5 } }'
```

## ImageXYawRegulate

A regulator built on of the `move` command that servos the robot's x & yaw according to object detections in an image.

A command example that regulate robot's rotation (yaw):
```bash
ros2 action send_goal /image_x_yaw_regulator/regulate spes_msgs/action/ImageXYawRegulate '{ "direction": 1, "tolerance": 0.02, "image_segment": 0.5 }'
```

A command example that regulate robot's position (x):
```bash
ros2 action send_goal /image_x_yaw_regulator/regulate spes_msgs/action/ImageXYawRegulate '{ "direction": 0, "tolerance": 0.05, "image_segment": 0.5 }'
```
