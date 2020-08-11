# mobile robot sim

## Rover Simulator
Thanks to the original developers of this project [@sravi2007](https://github.com/s2007ravi), [@yuthikasagarage](https://github.com/yuthikasagarage), and [@mturktest123](https://github.com/mturktest123), who's git contributions unfortunately eroded when performing a large refactor in order to support use for ROS-Agriculture general use. 

## Running
### Running from Docker
1. `docker build . -t mobile_robot_sim:v0.0`
2. `./docker_start.sh`

### Running on Bare Metal
1. Clone this repo into your catkin workspace src/
2. Install dependencies 
`sudo apt-get install ros-${ROS_DISTRO}-ompl #not sure why we have to do this manually
sudo apt-get install ros-${ROS_DISTRO}-mbf-msgs
rosdep update && \
rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
apt-get -qq upgrade`
3. `catkin build`

## Launch files
Currently only two launch files are being used.
1. Launch the robot:
`roslaunch rover_gazebo_control move_base_controller.launch`
2. Launch the controller / behavior tree
`roslaunch rover_gazebo_control start_navsat.launch`
