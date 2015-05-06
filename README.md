# Learning ROS for Robotics Programming - Second Edition #

**Learning ROS for Robotics Programming - Second Edition** book tutorials source code.

## Authors ##

* Aaron Martínez
* Anil Mahtani
* Enrique Fernández
* Luiz Sánchez

## Installation ##

Install **ROS Hydro** on a compatible **Ubuntu** distro following the official instructions provided [here](http://wiki.ros.org/hydro/Installation/Ubuntu).
Note that **ROS Indigo** might also work for most of the source code.

Create a workspace:
``` bash
mkdir -p ~/dev/catkin_ws/src
cd ~/dev/catkin_ws/src
wstool init
```

Download this repository:
``` bash
wstool set ROS_Book_Hydro --git git@github.com:AaronMR/ROS_Book_Hydro.git
wstool up -j8
```

Install the dependencies:
``` bash
cd ..
rosdep install --from-paths src -iy
```

source /opt/ros/$(rosversion -d)/setup.bash

Build the source code (alternatively, you can use `catkin build` instead of `catkin_make`):
``` bash
catkin_make -j4
source devel/setup.bash
```

## Tutorials ##

* **Chapter  1:** Getting started with ROS Hydro (no source code as it covers the installation)
* **Chapter  2:** ROS Architecture and Concepts
* **Chapter  3:** Visualization and Debug Tools
* **Chapter  4:** Using Sensors and Actuators with ROS
* **Chapter  5:** Computer Vision
* **Chapter  6:** Point Clouds
* **Chapter  7:** 3D Modeling and Simulation
* **Chapter  8:** The Navigation Stack - Robot Setup
* **Chapter  9:** The Navigation Stack - Beyond the Setup
* **Chapter 10:** Manipulation with MoveIt!
