### Cassidy Brown - cmb195
### 12/6/16

# ROS P9: Part Fetcher Action Server

P9 includes an action server that will pick up an ARIAC pulley

###Src Files:
  - part_fetcher_action_server
  	- Hybrid of part_fetcher/part_fetcher_exmpl and object_grabber/object_grabber_action_server
  	- Listens for pulley_fetcher_client on part_fetcher service
  	- Sends pickup and dropoff goals to object_grabber_action_server
  - pulley_fetcher_client
  	- adapted from part_fetcher/example_part_fetcher_client
  	- sends pickup and dropoff information to part_fetcher_action_server
  - object_manipulation_query_svc
  	- adapted from same-named file in object_manipulation_properties
  	- moved to this project for the sake of sticky_fingers
  - sticky_fingers_manip_fncs
  	- adapted from same-named file in object_manipulation_properties
  	- added functions for ARIAC_PULLEY
  	- needed to move it to this project so that the changes would be submitted with the assignments

###Launch File:
  - cwru_ariac.launch
  	- adapted from same in `cwru_ariac_launch` pkg
  	- modified to launch ur10_w_gripper (launches gazebo)
  	- calls p9_object_manipulation_query_svc rather than original  

### Execution
```
roslaunch p9 cwru_ariac.launch
roslaunch ariac_models add_parts.launch
rosrun p9 part_fetcher_action_server
rosrun p9 pullery_fetcher_client
```

    
