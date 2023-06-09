#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

#define rotation_error 0.2//radians

#define kpr 0.5
#define kir 0
#define kdr 0

class rotation {
private:

    ros::NodeHandle n;

    // communication with one_moving_person_detector or person_tracker
    ros::Subscriber sub_goal_to_reach;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    geometry_msgs::Point goal_to_reach;
    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not

    float translation_to_do;
    float rotation_to_do, rotation_done;
    bool cond_rotation;// boolean to check if we still have to rotate or not

    bool new_odom;//to check if  new data from odometry are available

    float init_orientation;
    float current_orientation;

    float error_integral;
    float error_previous;

    bool init_odom;
    bool display_odom;

public:

rotation() {

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &rotation::odomCallback, this);

    // communication with moving_persons_detector or person_tracker
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &rotation::goal_to_reachCallback, this);

    cond_rotation = false;
    new_goal_to_reach = false;
    init_odom = false;
    display_odom = false;

    error_integral = 0;
    error_previous = 0;

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we receive a new /goal_to_reach
    if ( new_goal_to_reach && init_odom ) {
        new_goal_to_reach = false;
        ROS_INFO("\n(rotation_node) processing the /goal_to_reach received from the perception node");
        ROS_INFO("(rotation_node) goal_to_reach: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        if ( translation_to_do )
        {
            //we compute the /rotation_to_do
            rotation_to_do = acos( goal_to_reach.x / translation_to_do );

            if ( goal_to_reach.y < 0 )
                rotation_to_do *=-1;

            init_orientation = current_orientation;
            rotation_done = current_orientation;
            rotation_to_do += current_orientation;
            cond_rotation = true;
            error_previous = rotation_to_do;

            if ( rotation_to_do > M_PI )
                rotation_to_do -= 2*M_PI;
            if ( rotation_to_do < -M_PI )
                rotation_to_do += 2*M_PI;
        }
        else
            rotation_to_do = 0;
    }

    //we are performing a rotation
    if ( init_odom && cond_rotation )
    {
        rotation_done = current_orientation;
        float error = ( rotation_to_do - rotation_done );

        if ( error > M_PI )
        {
            ROS_WARN("(rotation node) error > 180 degrees: %f degrees -> %f degrees", error*180/M_PI, (error-2*M_PI)*180/M_PI);
            error -= 2*M_PI;
        }
        else
            if ( error < -M_PI )
            {
                ROS_WARN("(rotation node) error < -180 degrees: %f degrees -> %f degrees", error*180/M_PI, (error+2*M_PI)*180/M_PI);
                error += 2*M_PI;
            }

        cond_rotation = ( fabs(error) > rotation_error );

        float rotation_speed = 0;
        if ( cond_rotation )
        {
            //Implementation of a PID controller for rotation_to_do;

            float error_derivation = error-error_previous;
            error_previous = error;
            //ROS_INFO("error_derivaion: %f", error_derivation);

            error_integral += error;
            //ROS_INFO("error_integral: %f", error_integral);

            //control of rotation with a PID controller
            rotation_speed = kpr * error + kir * error_integral + kdr * error_derivation;
            ROS_INFO("(rotation_action_node) current_orientation: %f, orientation_to_reach: %f -> rotation_speed: %f", rotation_done*180/M_PI, rotation_to_do*180/M_PI, rotation_speed*180/M_PI);
        }
        else
        {
            ROS_INFO("(rotation_node) current_orientation: %f, orientation_to_reach: %f -> rotation_speed: %f", rotation_done*180/M_PI, rotation_to_do*180/M_PI, rotation_speed*180/M_PI);
            rotation_done -= init_orientation;

            if ( rotation_done > M_PI )
                rotation_done -= 2*M_PI;
            if ( rotation_done < -M_PI )
                rotation_done += 2*M_PI;

            ROS_INFO("(rotation_node) final rotation_done: %f", rotation_done*180/M_PI);
            ROS_INFO("(rotation_node) waiting for a /goal_to_reach");
        }

        geometry_msgs::Twist twist;
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = rotation_speed;

        pub_cmd_vel.publish(twist);
    }

    //DISPLAY MSGS
    if ( !display_odom && !init_odom ) {
        ROS_INFO("wait for odom");
        display_odom = true;
    }
    if ( display_odom && init_odom )  {
        ROS_INFO("odom is ok");
        display_odom = false;
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "rotation");

    ROS_INFO("(rotation_node) waiting for a /goal_to_reach");
    rotation bsObject;

    ros::spin();

    return 0;
}
