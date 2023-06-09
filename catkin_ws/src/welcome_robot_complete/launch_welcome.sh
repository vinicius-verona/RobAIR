#!/bin/bash

#hacky script to save time...
#export DISPLAY=:1
echo "[Welcome_SN] Starting nodes..."

export SOCIAL_FOLLOWME_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $SOCIAL_FOLLOWME_DIR
source "$SOCIAL_FOLLOWME_DIR/../../../devel/setup.bash"

echo "[Welcome_SN] launching terminals..."

xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun rviz rviz -d /home/robair/.rviz/social_followme_laser_fixed_frame.rviz;/bin/bash" & 
#xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_social_follow sn_sf_control_node;/bin/bash" &
#xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_social_follow sn_sf_action_node_pid;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_social_follow sn_sf_robot_moving_node;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_social_follow sn_sf_obstacle_detection_node;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_social_follow sn_sf_detector_tracker_2lasers_node;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_social_follow sn_sf_decision_node;/bin/bash" &
#xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun --prefix 'gdb -ex run' sn_social_follow sn_sf_decision_node ;/bin/bash" &
#odom may be included in launch file already
#xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun odometry odometry_node _entrax:=35 _ticsPerMeter:=2452;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun audio_output audio_output_node;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun display_output display_output_node;/bin/bash" &

xterm -fa monaco -fs 13 -bg black -fg white -e "python ~/RobAIR/catkin_ws/src/robair_detection_navigation/src/robair_webcam_one.py;/bin/bash" &
#xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun robair_ps_teleop launch_teleop_joy.sh;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun robair_ps_teleop launch_teleop_joy_robot_only.sh ;/bin/bash" &
xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun welcome_robot_complete decision_welcome_robot_complete_node;/bin/bash" &


xinput=`xinput --list | grep 'Wacom'`
echo $xinput
if [ "$xinput" == "" ]
then
    echo "[Welcome_SN] Launching ui in xterm"
    xterm -fa monaco -fs 13 -bg black -fg white -e "rosrun sn_group_ui sn_group_ui;/bin/bash" &
else
    echo "[Welcome_SN] Launching ui in Xephyr session"
    Xephyr :1 -screen 1000x1000 &
    DISPLAY=:1 rosrun sn_group_ui sn_group_ui
fi


echo "[Welcome_SN] Robot is running. ctrl+c to kill all terminals"

input=""
while [ 1 ]
do
    read input
done


#cd catkin_ws/; source ./devel/setup.bash; rosrun wifibot_light wifibot_teleop_key.py


##useful stuff
#rosrun amcl amcl _odom_alpha1:=0.4 _update_min_=0.17 _odom_alpha2:=0.4
