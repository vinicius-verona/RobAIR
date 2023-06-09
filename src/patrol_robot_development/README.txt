first of all, move to ~/Robair/catkin_ws
run "robair start" to start roscore and robair

To run the welcome_robot, 5 nodes are needed:
see the image of the software architecture in software_architecture.png
1) rosrun welcome_robot decision_welcome_robot_node
=> to run the supervisor of the welcome robot

2 nodes for datmo:
2) rosrun welcome_robot robot_moving_welcome_robot_node
3) rosrun welcome_robot datmo_welcome_robot_node

2 nodes for control:
4) rosrun welcome_robot obstacle_detection_welcome_robot_node
5) rosrun welcome_robot action_welcome_robot_node

=> if there is a problem with the demo, you have to stop the action_welcome_robot_node

For the moment, robair doesnot go back to its base because aruco marker node is missing.
