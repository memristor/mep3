```bash
ros2 launch src/mep3/mep3_hardware/test/pumps/pumps_launch.py
ros2 action send_goal /big/pump/pump1 mep3_msgs/action/VacuumPumpCommand "{ connect: 1 }"
```