# Master thesis
## General information
### Important materials
1) [Presentation](https://drive.google.com/file/d/1pFDCmQryXQwL2Fri5JCQbI7lf9-F2rCw/view?usp=sharing)
2) [Dissertation](https://drive.google.com/file/d/1lgIQOYwCE5o1goZx6O_hlfct3TAkHzP4/view?usp=sharing)

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
├── [drone_controller](https://github.com/surfii3z/drone_controller/tree/thesis)
├── [Fast-Planner](https://github.com/surfii3z/Fast-Planner/tree/thesis)
├── [image_proc](https://github.com/surfii3z/image_proc/tree/thesis)
├── [image_undistort](https://github.com/surfii3z/image_undistort/tree/thesis)
└── [tello_driver](https://github.com/surfii3z/tello_driver/tree/thesis)
```
2) packnet_ros_ws

```bash
packnet_ros_ws/src/
├── [packnet_sfm_ros](https://github.com/surfii3z/packnet_sfm_ros)
└── vision_opencv
```

3) ORB_SLAM2

```bash
[ORB_SLAM2](https://github.com/surfii3z/ORB_SLAM2/tree/thesis)
````

## How to run
I experimented this system on `DGX-Station (V100)`, tested with `Ubuntu 18.04 LTS`, `python 3.6.9`, `TensorRT-7.1.3.4`, `PyTorch 1.4.0`, `Cuda 11.0` and `CuDNN 8.0.5`.
We need to use 8 terminals to run
```bash
# -------------------------
# 00_roscore
roscore

# -------------------------
# 01_depth_estimation_module
# pwd: /path/to/packnet_ros_ws
source install/setup.bash --extend
rosrun packnet_sfm_ros trt_packnet_node

# -------------------------
# 02_tello_driver
# pwd: /path/to/catkin_ws
source devel/setup.bash
roslaunch tello_driver thesis_tello_node.launch

# -------------------------
# 03_visual_SLAM
# pwd: /path/to/ORB_SLAM2/ros
source devel/setup.bash
roslaunch orb_slam2 rgbd_tello_crop.launch

# -------------------------
# 04_path_planner_visualization
# pwd: /path/to/catkin_ws
source devel/setup.bash
roslaunch plan_manage rviz.launch

# -------------------------
# 05_path_planner
# pwd: /path/to/catkin_ws
source devel/setup.bash
roslaunch plan_manage tello_kino_replan.launch


# -------------------------
# 06_drone_controller
# pwd: /path/to/catkin_ws
source devel/setup.bash
roslaunch drone_controller waypoint_controller.launch

# -------------------------
# 07_mission
# pwd: /path/to/catkin_ws
source devel/setup.bash
rosrun drone_controller thesis_mission.py
```
![terminal](https://github.com/surfii3z/jy_master_thesis/blob/main/media/terminal.png)

![rosgraph](https://github.com/surfii3z/jy_master_thesis/blob/main/media/rosgraph.png)

NOTE: The monocular depth estimation network must be trained based on the application.

## Results
Please see this [playlist](https://www.youtube.com/watch?v=TcpziH_DZm0&list=PLy765YYpYmKxbRtEeM9Om__sBgPfIyyZO) on Youtube
