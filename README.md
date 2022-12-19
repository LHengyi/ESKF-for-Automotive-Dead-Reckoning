# Demo: Carla Simulation of automotive Dead Recking Based-on Error State Kalman Filter
In this demo, a Kalman filter is applied to track the pose of an automotive. IMU data and speed information are collected through Carla rosbridge. In the update step, non-holonomic constraints are applied, e.g. assuming no jumping, no sliding.

## Prerequisite
* Linux and ROS (has been test on Ubuntu 20.04 with ROS Noetic).
* [Carla Simulator](http://carla.readthedocs.io/en/latest/start_quickstart/) and [Carla rosbridge](https://carla.readthedocs.io/en/0.9.10/ros_installation/)
## Build
* First, clone into you catkin workspace  
    cd catkin_ws/src   
    git clone https://github.com/LHengyi/ESKF-for-Automotive-Dead-Reckoning.git   
* Build  
    catkin_make   
* To run   
    roslaunch automotive_dead_reckoning carla_localization_ad_rosbridge.launch  
in another terminal  
    roslaunch automotive_dead_reckoning wheel_ins.launch  
## Demonstration
<!-- [Demo video](http:youtube) -->
<img src="image/carla_localization.gif" width="50%" height="50%"/>