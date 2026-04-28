To test mep3 vision, firstly start all nodes with launch.

```
ros2 launch mep3_vision vision_launch.py debug:=true
```

Then, in another terminal you can type:

```
ros2 action list
```

This will show all available actions, curently it will only show /big/camera and /big/aruco

---

In order to test action server you can type (in second terminal):

```
ros2 action send_goal /big/aruco mep3_msgs/action/Aruco "{camera_select: "front"}"
```

or

```
ros2 action send_goal /big/aruco mep3_msgs/action/Aruco "{camera_select: "back"}"
```

This will test aruco detection directly, response that you will get is 8-bit value wherethe first 4 LSB represent yellow hazlenuts that need to be flipped from left to right.

For example, if you have hazlenuts in [yellow, yellow, blue, blue] order (left to right from robot perspective) resposne that you will get is 00001100.
However, if there are any errors, 5th bit will be set to 1 (0001----)

---

In order to test table camera detection you can type (in second terminal):

```
ros2 action send_goal /big/camera mep3_msgs/action/Camera "{group_select: x}"
```

x is in range from 0 to 17. [0, 17]

Here you will get the group info which is a 32 bit value, however only first 18 LSB are important. If there are any hazlenuts in given area, the bit will be set to 1, 0 otherwise. Response is masked value to group info, should be above 0 if hazlenuts are detected (2^n will be the number) and exactly 0 if no hazlenuts are detected.
Here first 8 bits are set to 1 by default and rest are 0, in case of system error this will be the default value that should not impact any strategies.

---