# Master thesis
## Information
### Title
The development of the scale-aware monocular depth estimation aided monocular visual SLAM system for real-time robot navigation

### Motivation
Human pilots could fly any drone by relying only on the monocular visual signal in the goggle. In this work, we want to use high performance computer to replace the human brain to navigate the drone by relying only on the monocular camera sensor.

### Objective
To enable the mobile robot to autonomously navigate in an unknown environment using only a monocular camera as its sensor

### Approaches
1) `RGB-Deep D visual SLAM`: Augmented the `monocular depth estimation` (Packnet-sfm) to provide the pseudo depth information to the monocular visual SLAM (ORB_SLAM2) to solves its inherent problems: scale-ambiguity, tracking robustness, and map initialization delay (See presentation file page 5).
