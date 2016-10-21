## Cassidy Brown - cmb195
### ROS P6 - 10/20/16

This package contains a bag of lidar data points and a node which interprets them.
The node was taken from Part_3/lidar_wobbler in learning_ros, but the rest of that package was not relevant to this assignment and is not included here.

Node:
    lidar_transformer_p6

Changes to node compared to lidar_wobbler implementation:
  - ~line 75 analyzing points to interpret shape involves significantly more analysis now
  - Added global variables to track dimensions of the block
  - Added blockInfo function to print calculations 


To run:
  > roscore
  > rosparam set use_sim_time true				# not entirely sure what that is, but I'll assume it's important
  > rosrun rviz rviz
  > rosrun p6 lidar_transformer_p6
  > rosbag play block_scan.bag --clock

