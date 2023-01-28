# Test Dynamixel Interface

```bash
# Start the interface and the controller
ros2 launch src/mep3/mep3_hardware/test/dynamixel/dynamixel_joint_trajectory_launch.py

# Send some command to the controller
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1],
    points: [
      { positions: [1], time_from_start: { sec: 1 } },
      { positions: [-1], time_from_start: { sec: 2 } },
      { positions: [0], time_from_start: { sec: 3 } }
    ]
  }
}"

# Or faster
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory -f "{
  trajectory: {
    joint_names: [joint1],
    points: [
      { positions: [1], time_from_start: { nanosec: 300000000 } },
      { positions: [-1], time_from_start: { nanosec: 600000000 } },
      { positions: [0], time_from_start: { nanosec: 900000000 } }
    ]
  }
}"
```


```bash
# Start the interface and the controller
ros2 launch src/mep3/mep3_hardware/test/dynamixel/dynamixel_launch.py

# Send some command to the controller
ros2 action send_goal /joint_position_command/joint1 mep3_msgs/action/JointPositionCommand -f "{ position: 1.0, max_velocity: 0.5 }"
```