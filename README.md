# Learning ROS for Robotics Programming - Second Edition #

[**Learning ROS for Robotics Programming - Second Edition**](https://www.packtpub.com/hardware-and-creative/learning-ros-robotics-programming-%E2%80%93-second-edition) book tutorials source code.

<a href="https://www.packtpub.com/hardware-and-creative/learning-ros-robotics-programming-%E2%80%93-second-edition"><img src=https://www.packtpub.com/sites/default/files/7580OS_Learning%20ROS%20for%20Robotics%20Programming%20-%20Second%20Edition.jpg width=200/></a>

## Authors ##

* [Enrique Fernández](https://github.com/efernandez)
* [Luis Sánchez Crespo](https://github.com/LuisSC)
* [Anil Mahtani](https://github.com/Anilm3)
* [Aaron Martínez](https://github.com/AaronMR)

## Installation ##

Install **ROS Jade** on a compatible **Ubuntu** distro following the official instructions provided [here](http://wiki.ros.org/hydro/Installation/Ubuntu).

For **ROS Hydro** use the [**hydro-devel**](https://github.com/AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition/tree/hydro-devel) branch.

For **ROS Indigo** use the [**indigo-devel**](https://github.com/AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition/tree/indigo-devel) branch.

Install the OpenCV non-free repository:
``` bash
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get install libopencv-nonfree-dev libopencv-nonfree2.4
```

Create a workspace:
``` bash
mkdir -p ~/dev/catkin_ws/src
cd ~/dev/catkin_ws/src
wstool init
```

Download this repository:
``` bash
wstool set ros_book --git git@github.com:AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition.git
wstool up -j8
```

Install the dependencies:
``` bash
cd ..
rosdep install --from-paths src -iy
```

Build the source code (alternatively, you can use `catkin build` instead of `catkin_make`):
``` bash
source /opt/ros/$(rosversion -d)/setup.bash
catkin_make -j4
source devel/setup.bash
```

## Tutorials ##

* **Chapter  1:** Getting started with ROS (no source code as it covers the installation)
* **Chapter  2:** ROS Architecture and Concepts
* **Chapter  3:** Visualization and Debug Tools
* **Chapter  4:** Using Sensors and Actuators with ROS
* **Chapter  5:** Computer Vision
* **Chapter  6:** Point Clouds
* **Chapter  7:** 3D Modeling and Simulation
* **Chapter  8:** The Navigation Stack - Robot Setup
* **Chapter  9:** The Navigation Stack - Beyond the Setup
* **Chapter 10:** Manipulation with MoveIt!
