# rrt_motion_plan
This repo contains a ROS-Rviz based implemntation of the RRT(Rapidly exploring random tress) algorithm

## How to run?

Clone the repo to your local catkin package and then build it from the top of your package through ``catkin_make``.
  After buildinfg the package type in the following commands in separate terminals.
   
   ``roscore`` for bringing up the ROS master.
    
  `` roslaunch rrt_motion_plan rrt.launch`` for loading the paramaters to the parameter server.
  
  `` rosrun rviz rviz `` for starting rviz, subsequently add the marker visual type from the option window.
  
  `` rosrun rrt_motion_plan rrt`` for starting the rrt node.
  
## Setting up the rrt config through yaml file

All the required rrt configurations can be set through the  ``rrt_config.yaml`` file. These include obstacle numbers, obstacle position, start position and end position.
