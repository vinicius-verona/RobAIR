geometry_msgs/Point goal_to_reach     # aruco position
bool apf_in_execution                 # Tells if APF algorithm is being executed
                                      # if true, the next messages should be provided
                                      # to obstacle_avoidance and decision node should lock the process