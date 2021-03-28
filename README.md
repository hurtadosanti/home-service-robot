# Home Service Robot

On this Udacity project we simulate a full home service robot capable of navigating to pick up and deliver virtual objects. The robot we use is the turtlebot provided by ROS. The robot is capable to localize, map and navigate using a fusion of LIDAR, camera and odometer. We have used the SLAM_gmapping module and the AMCL adaptive Monte Carlo localization provided by ROS.

---

## Introduction
### Localization and Mapping
GMapping is a highly efficient Rao-Blackwellized particle filer to learn grid maps from laser range data. Using SLAM_gmapping, you can create a 2-D occupancy grid map from laser and pose data collected by the robot.

### Navigation
For navigation and object avoidance we use the ROS Navigation stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm.

---

## Dependencies
- Ubuntu 16.04
- ROS Kinetic Kame
- [AMCL](http://wiki.ros.org/amcl) ROS Module
- [slam_gmapping](http://wiki.ros.org/slam_gmapping)


## Installation
- Create a workspace

        mkdir -p catkin_ws
        cd catkin_ws

- Clone this repository on the src folder location
  
        git clone --recursive git@github.com:hurtadosanti/home-service-robot.git ./src

- Initialize workspace
        
        cd src
        catkin_init_workspace

- Build
  
        cd ..
        catkin_make

## Execution
Run the main program in a terminal with X support
  
    cd catkin_ws
    source devel/setup.bash
    ./src/scripts/home_service.sh


## License
MIT License Copyright (c) 2021 Santiago Hurtado
