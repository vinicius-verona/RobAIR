if [[ -v ROS_MASTER_URI ]]; then
    echo deploying code to robair with IP => $ROS_MASTER_URI
    ros_ip_address=$(echo "$ROS_MASTER_URI" | cut -d'/' -f3 | cut -d':' -f1)
    ping -q -W 5 -c 5 $ros_ip_address 2>&1 > /dev/null || { echo "Not connected to RobAIR network!";}
    scp -r $HOME/RobAIR-DEV/RobAIR robair@$ROS_MASTER_URI:~/vinicius-patrol-robot-development/RobAIR
    ssh -t robair@$ROS_MASTER_URI "sh update_catkin_package.sh && cd $HOME/RobAIR/catkin_ws && catkin_make"

else
    echo "ROS_MASTER_URI must be set as an environment variable in the following format: http://<ip_address>:<port>"
fi