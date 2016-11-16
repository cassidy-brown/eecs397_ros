Cassidy Brown - cmb195
ROS p7
10/28/16

Code taken from learning_ros/Part_5/baxter/baxter_playfile_nodes

The only joint state files are ymca_r and ymca_l, which are meant to be called together and control the right and left arms respectively

Baxter does the YMCA when you run: 
	roslaunch baxter_gazebo baxter_world.launch
	roslaunch baxter_launch_files baxter_playfile_service_nodes.launch
	rosrun p7 baxter_playback_p7 ymca_r.jsp ymca_l.jsp