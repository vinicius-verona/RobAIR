#!/bin/bash

if [[ -n "$ROS_MASTER_URI" ]]; then
    echo "Deploying code to robair with IP ($ROS_MASTER_URI)"
    ros_ip_address=$(echo "$ROS_MASTER_URI" | cut -d'/' -f3 | cut -d':' -f1)
    
    if ping -q -W 5 -c 5 $ros_ip_address >/dev/null; then
        rsync -av --exclude='.gitignore' --exclude='.git' $HOME/RobAIR-DEV/RobAIR robair@$ros_ip_address:~/vinicius-patrol-robot/
        ssh -t robair@$ros_ip_address "sh ~/vinicius-patrol-robot/update_catkin_package.sh"
    else
        echo "Not connected to RobAIR network!"
    fi

else
    echo "ROS_MASTER_URI must be set as an environment variable in the following format: http://<ip_address>:<port>"
fi
