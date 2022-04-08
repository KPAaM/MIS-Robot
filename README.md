# MIS_franka

### Setup
1. ROS Noetic
2. MoveIt! -> `sudo apt install ros-noetic-moveit`
3. libfranka + franka_ros -> `sudo apt install ros-noetic-libfranka ros-noetic-franka-ros`
4. Clone this repository

    `git clone https://github.com/KPAaM/MIS-Robot.git`

    or
    
    `git clone git@github.com:KPAaM/MIS-Robot.git`
5. Build your workspace -> `catkin_make`


### ROS Master-Slave connection
1. Set IP address on your **slave PC** `192.16.0.2`
2. Set these two commands in `~/.bashrc` on your **slave PC**:

    `export ROS_MASTER_URI=http://192.16.0.1:11311`

    `export ROS_IP=192.16.0.2`

### Connect to the master PC
`ssh -X km@192.16.0.1`

After logging into the master PC you need to release the robot's brakes and allow FCI. By writting `firefox` into terminal,
you will automatically enter `Franka DESK` where you can do these steps.


### Start guide
1. Launch Franka cartesian_velocity_controller and force_controller on **master PC**:

    `roslaunch franka_mis_controllers franka_mis_controllers.launch`

2. Launch joystick node (joystick driver + joystick_EEF_controller) on **slave PC**:

    `roslaunch franka_joystick_control joystick_control.launch`

### Important topics
`/joystick_feedback` -> Raw data from joystick. For more info check `/franka_joystick_control/src/joystick_pub.cpp`

`/joystick_cartesian_goal` -> Commands for Franka representing cartesian velocities



### Acknowledgement
[franka_ros](https://github.com/frankaemika/franka_ros)


