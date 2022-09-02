- [ ] Upgrade to Humble.
- [x] Utilize Composable Nodes in Nav2.
- [ ] Investigate where is it possible to utilize Composable Nodes.
- [ ] Reimplement laser_inflator as a costmap layer. The layer should "paint" extended number of pixels with the `setConvexPolygonCost` method. Or, maybe simply check whether we can utilize the inflation_layer.
- [ ] Reimplement distance_angle_controller as Nav2 plugins (a controller and global planner).
- [ ] Reimplement dynamixel node as a ros2_control hardware interface (+ controller).
- [ ] Simplify the simulation so only the base has physics properties (with passive wheels). 
- [ ] Cleanup unused Nav2 plugins and remove the map server.

- [ ] Measure startup performance on Raspberry Pi
