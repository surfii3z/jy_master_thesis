# Master thesis
## General information
### Title
The development of the scale-aware monocular depth estimation aided monocular visual SLAM system for real-time robot navigation

### Motivation
Human pilots could fly any drone by relying only on the monocular visual signal in the goggle. In this work, we want to use high performance computer to replace the human brain to navigate the drone by relying only on the monocular camera sensor.

### Objective
To enable the mobile robot to autonomously navigate in an unknown environment using only a monocular camera as its sensor

### Approaches
1) `RGB-Deep D visual SLAM`: Augmented the `monocular depth estimation` (Packnet-sfm) to provide the pseudo depth information to the monocular visual SLAM (ORB_SLAM2) to solves its inherent problems: scale-ambiguity, tracking robustness, and map initialization delay (See presentation file page 5).

![RGB_Deep_D_visual_SLAM](https://github.com/surfii3z/jy_master_thesis/blob/main/media/RGB_Deep_D_visual_SLAM.png)

2) `Navigation stack`: Following the `see-think-act` scheme for autonomous mobile robot, we integrated visual SLAM (see, `RGB-Deep D SLAM`, _localization/ mapping_), local path planner (think, `Fast-Planner`, _planning/ obstacle avoidance_), and waypoint tracking controller (act, `PD controller`, _control_).

Module | Employed system
------------ | -------------
Visual SLAM | [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
Monocular Depth Estimation | [packnet-sfm](https://github.com/TRI-ML/packnet-sfm)
Local path planner | [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
Controller | [PD controller](https://github.com/surfii3z/drone_controller/tree/thesis)

![see_think_act](https://github.com/surfii3z/jy_master_thesis/blob/main/media/see_think_act.png)

![Navigation_stack](https://github.com/surfii3z/jy_master_thesis/blob/main/media/Navigation_stack.png)

## Dependencies
We make `3` seperated workspace called 
1) catkin_ws
```bash
catkin_ws/src
├── drone_controller
├── Fast-Planner
├── image_proc
├── image_undistort
├── robot_localization
└── tello_driver
```
2) orb_slam_ws

```bash
orb_slam_ws/
└── orb_slam2
````

3) packnet_ros_ws

```bash
packnet_ros_ws/src/
├── packnet_sfm_ros
└── vision_opencv
```
