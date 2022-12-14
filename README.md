# drone_trajectory_control
Robust Trajectory Tracking for Quadrotor UAVs using Sliding Mode Control

## Repository Setup
In order to setup the repository locally on your system, open a new terminal and follow the instructions below:

    cd ~
    mkdir -p rc_project/src
    cd rc_project/src
    git clone git@github.com:dennyboby/drone_trajectory_control.git
    cd ..
    catkin init
    catkin build -j4
    source devel/setup.bash

## Run Trajectory Generation
In order to run the trajectory generation run the matlab code `rc_project.m` which is present in `rc_project/src/drone_trajectory_control/matlab`.