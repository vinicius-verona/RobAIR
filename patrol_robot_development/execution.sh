#!/bin/bash

# Open a new terminal window
function open_terminal() {
    gnome-terminal --window --wait -- bash -c "echo $1 && $SHELL"  &
    if [ "$1" = "roscore" ]; then
        sleep 3
    else
        sleep 1.5
    fi
}

# Move terminal window to a specific workspace
function move_to_workspace() {
    local wid=$1
    local workspace=$2
    wmctrl -i -r $wid -t $workspace
}

# Set terminal window position and size
function set_window_position() {
    local wid=$1
    local x=$2
    local y=$3
    local width=$4
    local height=$5
    local new_name=$6

    # Set the name of the window and new position
    xdotool set_window --name "$new_name" "$wid"
    wmctrl -i -r $wid -e 0,$x,$y,$width,$height
}

# Get the window ID of the last opened terminal
function get_last_window_id() {
    wmctrl -l | awk '{print $1}' | tail -n 1
}

# Calculate screen width and height
screen_width=$(xdpyinfo | awk '/dimensions/{print $2}' | cut -d 'x' -f 1)
screen_height=$(xdpyinfo | awk '/dimensions/{print $2}' | cut -d 'x' -f 2)

# Calculate window width and height
window_width=$((screen_width / 2))
window_height=$((screen_height / 2))


if dpkg -s wmctrl &> /dev/null && dpkg -s xdotool &> /dev/null; then
    # Open terminals in the first workspace - Basic commands
    open_terminal "./switch_config.sh robair"
    wid=$(get_last_window_id)
    move_to_workspace $wid 1
    set_window_position $wid 0 0 $window_width $window_height  "Switch Config"

    open_terminal "roscore"
    wid=$(get_last_window_id)
    move_to_workspace $wid 1
    set_window_position $wid $window_width 0 $window_width $window_height "Roscore"

    open_terminal "roslaunch robairmain robair_ufr.launch"
    wid=$(get_last_window_id)
    move_to_workspace $wid 1
    set_window_position $wid 0 $window_height $window_width $window_height "Roslaunch"

    open_terminal "rosrun robair_detection_navigation robair_webcam_one.py"
    wid=$(get_last_window_id)
    move_to_workspace $wid 1
    set_window_position $wid $window_width $window_height $window_width $window_height "Rosrun Webcam"

    # Open terminals in the second workspace - Follow-me
    open_terminal "rosrun patrol_robot_development obstacle_detection_patrol_robot_development_node"
    wid=$(get_last_window_id)
    move_to_workspace $wid 2
    set_window_position $wid 0 0 $window_width "$(($window_height * 2))" "Obstacle Detection"

    open_terminal "rosrun patrol_robot_development robot_moving_patrol_robot_development_node"
    wid=$(get_last_window_id)
    move_to_workspace $wid 2
    set_window_position $wid $window_width 0 $window_width "$(($window_height * 2))" "Robot Moving"

    # Open terminals in the third workspace - Welcome-Complete
    open_terminal "rosrun patrol_robot_development datmo_patrol_robot_development_node"
    wid=$(get_last_window_id)
    move_to_workspace $wid 3
    set_window_position $wid 0 0 $window_width $window_height "Datmo Welcome"

    open_terminal "rosrun patrol_robot_development decision_patrol_robot_development_vinicius_node"
    wid=$(get_last_window_id)
    move_to_workspace $wid 3
    set_window_position $wid $window_width 0 $window_width $window_height "Decision Welcome"

    open_terminal "rosrun patrol_robot_development rotation_patrol_robot_development_node"
    wid=$(get_last_window_id)
    move_to_workspace $wid 3
    set_window_position $wid 0 $window_height $window_width $window_height "Rotation Welcome"

    open_terminal "rosrun patrol_robot_development action_patrol_robot_development_node"
    wid=$(get_last_window_id)
    move_to_workspace $wid 3
    set_window_position $wid $window_width $window_height $window_width $window_height "Action Welcome"
else
    apt-get install wmctrl  -y
    apt-get install xdotool -y
fi
