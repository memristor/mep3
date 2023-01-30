# Move Behavior

Provides primitive but flexible robot motion behavior.

## Frames

There are four important frames:
- We defined an objective as a `target` frame (2D pose) in the `global` frame.
- The robot (`base` frame) finds the target in the `odom` frame and continues motion in the `odom` frame.

Therefore, there are two stages:
- Once the command is received we calculate the `target` pose in the `odom` frame: $$ T_{odom}^{target} = (T_{global}^{odom})^{-1} * T_{global}^{target} $$
- In the control loop we regulate the position according to the error: $$ T_{base}^{target} = (T_{odom}^{base})^{-1} * T_{odom}^{target} $$

In the typical example `global` = `map` and `odom` = `odom`.
It means the robot moves to the absolute pose, but it uses the `odom` frame to avoid discrete pose jumps.

However, `global` + `odom` frames gives us a flexibility to achieve useful behaviors:
- If `global` = `base_link` and `odom` = `odom` then the robot moves relative to its current pose.
- If `global` = `marker_1` (a fiducial marker) and `odom` = `marker_1` then the robot moves to the marker pose (e.g. to pick the object). 
- However, if the robot loses the `marker_1` from its sight then we can set the following `global` = `marker_1` and `odom` = `odom`.

## Regulation

TODO

## Examples

```
ros2 action send_goal /big/move_behavior mep3_msgs/action/Move "{ target: { theta: 3.13 }, header: { frame_id: 'map' }, odom_frame: 'odom', angular_properties: { kp: 20, max_velocity: 1.5, max_acceleration: 0.8, tolerance: 0.01 }, ignore_obstacles: true }"
```
