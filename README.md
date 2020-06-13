# RoverROS

A repository containing the source code of a Rover with a 5DOF manipulator.

## Dependancies:

You will need to install the following:

  ROS Melodic
  
  `sudo apt-get install ros-melodic-ackermann-msgs`

  `sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`
  
  `sudo apt-get install ros-melodic teleop-twist-keyboard Keyboard.py`

##How to Build the Repo (Fresh Start)

    1) Git clone https://github.com/mturktest123/RoverYuthika
    2) Cd RoverYuthika
    3) Git checkout thisisworking
    4) Catkin init
    5) Catkin clean
    6) Catkin_make
    7) source devel/setup.bash

##Launching Files For GPS Coordinate Navigation
These launch files need to be executed in separate terminals 

Each terminal should be sourced with this command -> source devel/setup.bash

    `In the rover_gazebo_control package, waypoints.txt file is there, dummy waypoints are already there for testing`
    To change the waypoints you need to give latitude and longitude coordinates around 49.9 and 8.9
    If the given lat/lon is far the global planner will not be able to generate a path.


    
    1) roslaunch rover_desc_pkg move_base_controller.launch
    2) roslaunch rover_gazebo_control start_navsat.launch
    3) roslaunch rover_gazebo_control gps_waypoint_nav.launch
    
##Launching Files for Tele-Operation 
These commands need to be executed in separate terminals

Each terminal should be sourced with this command -> source devel/setup.bash

This will allow you to teleoperate the rover as you play a need for speed game :D

    1) roslaunch rover_desc_pkg rover_gazebo_control.launch
    1) rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    2) roslaunch rover_desc_pkg teleop.launch
    
