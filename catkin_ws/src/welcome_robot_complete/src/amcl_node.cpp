#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "std_msgs/Bool.h"

#define frequency_expected 25

class amcl_node {
private:

    ros::NodeHandle n;

    // communication with amcl
    ros::Subscriber sub_amcl;

public:

amcl_node() {

    // communication with amcl
    sub_amcl = n.subscribe("amcl_pose", 1, &amcl_node::localizationCallback, this);

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok())
    {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update()
{



}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& l)
{
// process the localization received from amcl_pose

    new_localization = true;
    localization = l->pose.pose.position;
    orientation = tf::getYaw(l->pose.pose.orientation);

}

};

int main(int argc, char **argv){

    ROS_INFO("(amcl_node) waiting for a /goal_to_reach");
    ros::init(argc, argv, "amcl_node");

    amcl_node bsObject;

    ros::spin();

    return 0;
}
