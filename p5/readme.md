# 3-DOF Robot

Cassidy Brown - cmb195
ROS P5 submission
10/13/16

This package includes a robot with three joints, three_DOF_robot, as described in the urdf.
We use the gazebo controller plugin and specify controllers in the provided YAML.
The controllers are commanded via a sine commander node, which publishes sins of differing frequencies to control the robot's three joints.


URDF:
	minimal_robot_description.urdf
YAML:
	three_dof_ctl_params.yaml
Node:
	sin_commander_p5
Launch:
	three_dof_jnt_ctl.launch

