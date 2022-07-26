# Mobile-Robot-Autonomous-Navigation
Course Project of Design And Practice of Robot of Zhejiang University.

## **Contents**
- [Introduction](#introduction)
- [Development Environment](#development-environment)
- [Application Scenarios](#application-scenarios)

<hr>

### **Introduction**
This repo implements a complete EKF-SLAM navigation system. The main feature is completing mobile robot autonomous navigation and build the environment map and feature map when the environment map is completely unknown.
```
code:
.
├── course_agv_control
│   ├── CMakeLists.txt
│   ├── config
│   │   └── course_agv_control.yaml
│   ├── launch
│   │   └── course_agv_control.launch
│   ├── package.xml
│   └── scripts
│       ├── keyboard_velocity.py
│       └── kinematics.py
├── course_agv_description
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── course_agv.rviz
│   │   └── course_agv_rviz.launch
│   ├── meshes
│   │   └── hokuyo.dae
│   ├── package.xml
│   └── urdf
│       ├── course_agv.gazebo
│       ├── course_agv.xacro
│       └── materials.xacro
├── course_agv_gazebo
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── map
│   │   └── map.yaml
│   ├── launch
│   │   ├── course_agv.rviz
│   │   ├── course_agv_world.launch
│   │   └── course_agv_world_rviz.launch
│   ├── models
│   │   └── ground_plane_for_agv
│   │       ├── map
│   │       │   ├── map.png
│   │       │   └── map_pillar.png
│   │       ├── materials
│   │       │   └── textures
│   │       │       ├── flat_normal.png
│   │       │       └── grey.png
│   │       ├── model.config
│   │       └── model.sdf
│   ├── package.xml
│   ├── scripts
│   │   └── robot_tf.py
│   └── worlds
│       └── course_agv.world
├── course_agv_nav
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── myreplan.launch
│   │   ├── nav.launch
│   │   └── nav.rviz
│   ├── package.xml
│   ├── scripts
│   │   ├── a_star.py
│   │   ├── a_star.pyc
│   │   ├── collision_checker.py
│   │   ├── dwa.py
│   │   ├── dwa.pyc
│   │   ├── global_planner.py
│   │   ├── local_planner.py
│   │   └── stupid_tracking.py
│   └── srv
│       └── Plan.srv
└── course_agv_slam_task
    ├── CMakeLists.txt
    ├── launch
    │   ├── all.launch
    │   ├── course_agv.rviz
    │   ├── ekf.launch
    │   ├── ekf_all.launch
    │   ├── extraction.launch
    │   ├── icp.launch
    │   ├── icp_all.launch
    │   ├── icp_lm.launch
    │   ├── mapping.launch
    │   └── particle_filter.launch
    ├── package.xml
    └── src
        ├── ekf.cpp
        ├── extraction.cpp
        ├── icp.cpp
        ├── icp_lm.cpp
        ├── mapping.cpp
        └── particle_filter.cpp
```

<hr>

### **Development Environment**
- Ubuntu 18.04
- ROS Melodic
- Python 2.7
- C++ 17

<hr>

### **Application Scenarios**
### **1. Simple Navigation**
Only the initial and destination coordinates are known.

<video src="./video/simple navigation.mp4" controls="controls" width=450> </video>

### **2. Dynamic Environment**
Only the initial and destination coordinates are known.
Dynamically add obstacle while robot is navigating.

<video src="./video/dynamic environment.mp4" controls="controls" width=450> </video>

### **3. Replanning**
Only the initial and destination coordinates are known.
The original planned path is blocked to test the replanning ability.

<video src="./video/replanning.mp4" controls="controls" width=450> </video>
