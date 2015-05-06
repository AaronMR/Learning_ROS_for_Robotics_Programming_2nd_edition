# ROS Book Hydro - Chapter 10 tutorials : ROS arm in MoveIt! #

## Launch MoveIt! ##

* For a kinematic version (very fast) without controllers:

``` bash
roslaunch rosbook_arm_moveit_config demo.launch
```

[![rosbook_arm_moveit_no_controllers.png](https://raw.githubusercontent.com/AaronMR/ROS_Book_Hydro/master/chapter10_tutorials/images/rosbook_arm_moveit_no_controllers.png)](http://youtu.be/aAihbFjSwBo)

* For a version using the simulated model of the arm:

``` bash
roslaunch rosbook_arm_gazebo rosbook_arm_empty_world.launch
roslaunch rosbook_arm_moveit_config moveit_rviz.launch config:=true
```

[![rosbook_arm_moveit.png](https://raw.githubusercontent.com/AaronMR/ROS_Book_Hydro/master/chapter10_tutorials/images/rosbook_arm_moveit.png)](https://youtu.be/gZJDvElwqg0)

# Pick & Place #

Run the simulated model of the arm on the grasping world:

``` bash
roslaunch rosbook_arm_gazebo rosbook_arm_grasping_world.launch
roslaunch rosbook_arm_moveit_config moveit_rviz.launch config:=true
roslaunch rosbook_arm_pick_and_place grasp_generator_server.launch
rosrun rosbook_arm_pick_and_place pick_and_place.py
```

This [video](http://youtu.be/GR0pmhgVq70) shows the grasping world, which also
spawns the arm with an RGB-D sensor on top of it.

Note that the objects in the planning scene has been created manually, without
perception, by using their dimensions from the SDF files in `~/.gazebo/models`
or the `meshes/<model>.dae` in meshlab as with the coke_can model.

## Demo mode ##

Run the following instead:

``` bash
roslaunch rosbook_arm_moveit_config demo.launch
roslaunch rosbook_arm_pick_and_place grasp_generator_server.launch
rosrun rosbook_arm_pick_and_place pick_and_place.py
```

This [video](http://youtu.be/q2YBhHWuJS0) shows the arm doing a Pick and Place
in demo mode.
