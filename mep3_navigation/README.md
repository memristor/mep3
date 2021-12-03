# mep3_navigation

Contains Navigation2 plugins and configuration files, and custom ROS 2 navigation nodes. 

## Distance-angle regulator

A custom ROS 2 node that regulates distance and angle to reach a target.
Generates trajectory with trapezoidal velocity shape.
Based on part from work ["On-Line Planning of Time-Optimal, Jerk-Limited Trajectories"](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.561.9756&rep=rep1&type=pdf) by Robert Haschke, Erik Weitnauer and Helge Ritter.
