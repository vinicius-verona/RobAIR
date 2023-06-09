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

#define rotation_error 0.1//radians
#define rotation_speed_max 0.5//radians
#define translation_error 0.1// meters

#define kpr 0.5
#define kir 0
#define kdr 0

#define kpt 0.5
#define kit 0
#define kdt 0

#define security_distance 0.5

class action {
private:

    ros::NodeHandle n;

    // communication with one_moving_person_detector or person_tracker
    ros::Subscriber sub_goal_to_reach;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    geometry_msgs::Point goal_to_reach;
    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not

    bool cond_goal;// boolean to check if we still have to reach the goal or not

    //pid for rotation
    float rotation_to_do, rotation_done;
    float error_rotation;//error in rotation
    bool cond_rotation;//boolean to check if we still have to rotate or not
    float initial_orientation;// to store the initial orientation ie, before starting the pid for rotation control
    float current_orientation;// to store the current orientation: this information is provided by the odometer
    float error_integral_rotation;
    float error_previous_rotation;
    float rotation_speed;

    //pid for translation
    float translation_to_do, translation_done;
    float error_translation;//error in translation
    bool cond_translation;//boolean to check if we still have to translate or not
    geometry_msgs::Point initial_position;// to store the initial position ie, before starting the pid for translation control
    geometry_msgs::Point current_position;// to store the current position: this information is provided by the odometer
    float error_integral_translation;
    float error_previous_translation;
    float translation_speed;

    bool init_odom;
    bool init_obstacle;
    geometry_msgs::Point closest_obstacle;   

public:

action() {

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &action::odomCallback, this);

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &action::closest_obstacleCallback, this);

    // communication with datmo
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &action::goal_to_reachCallback, this);

    cond_goal = false;
    new_goal_to_reach = false;
    init_odom = false;   
    init_obstacle = false;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
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

    if ( init_odom && init_obstacle ) { //wait for the initialization of odometer and detect_obstacle_node

        // we receive a new /goal_to_reach
        if ( new_goal_to_reach )
            init_action();

        //we are performing a rotation and a translation
        if ( cond_goal )
        {
            perform_rotation();
            perform_translation();
            combine_rotation_and_translation();
            move_robot();
        }
    }
    else
        if ( !init_obstacle )
            ROS_INFO("waiting for obstacle_detection_node");

}// update

void init_action()
{
    new_goal_to_reach = false;
    ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

    if ( translation_to_do )
    {
        cond_goal = true;

        //we compute the /rotation_to_do
        rotation_to_do = acos( goal_to_reach.x / translation_to_do );

        if ( goal_to_reach.y < 0 )
            rotation_to_do *=-1;

        //we initialize the pid for the control of rotation
        initial_orientation = current_orientation;
        error_integral_rotation = 0;
        error_previous_rotation = 0;

        //we initialize the pid for the control of translation
        initial_position = current_position;
        error_integral_translation = 0;
        error_previous_translation = 0;

        ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
        ROS_INFO("initial_orientation: %f, initial_position: (%f, %f) provided by odometer", initial_orientation*180/M_PI, initial_position.x, initial_position.y);

    }
    else
        ROS_WARN("translation_to_do is equal to 0");

}// init_action

void perform_rotation()
{

    ROS_INFO("current_orientation: %f, initial_orientation: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI);
    rotation_done = current_orientation - initial_orientation;

    //do not forget that rotation_done must always be between -M_PI and +M_PI
    if ( rotation_done > M_PI )
    {
        ROS_WARN("rotation_done > 180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done-2*M_PI)*180/M_PI);
        rotation_done -= 2*M_PI;
    }
    else
        if ( rotation_done < -M_PI )
        {
            ROS_WARN("rotation_done < -180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done+2*M_PI)*180/M_PI);
            rotation_done += 2*M_PI;
        }

    error_rotation = ( rotation_to_do - rotation_done );
    ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f", rotation_to_do*180/M_PI, rotation_done*180/M_PI, error_rotation*180/M_PI);

    cond_rotation = ( fabs(error_rotation) > rotation_error );

    rotation_speed = 0;
    if ( cond_rotation )
    {
        //Implementation of a PID controller for rotation_to_do;int

        float error_derivation_rotation = error_rotation-error_previous_rotation;
        error_previous_rotation = error_rotation;
        //ROS_INFO("error_derivation_rotation: %f", error_derivation_rotation);

        error_integral_rotation += error_rotation;
        //ROS_INFO("error_integral_rotation: %f", error_integral_rotation);

        //control of rotation with a PID controller
        rotation_speed = kpr * error_rotation + kir * error_integral_rotation + kdr * error_derivation_rotation;
        ROS_INFO("rotation_speed: %f", rotation_speed*180/M_PI);
    }
    else
        ROS_WARN("pid for rotation will stop");

}//perform_rotation

void perform_translation()
{

    ROS_INFO("current_position: (%f, %f), initial_position: (%f, %f)", current_position.x, current_position.y, initial_position.x, initial_position.y);
    translation_done = distancePoints( initial_position, current_position );
    error_translation = translation_to_do - translation_done;

    ROS_INFO("translation_to_do: %f, translation_done: %f, error_translation: %f", translation_to_do, translation_done, error_translation);

    cond_translation = ( fabs(error_translation) > translation_error );

    translation_speed = 0;

    if ( cond_translation )
    {
        //Implementation of a PID controller for translation_to_do;

        float error_derivation_translation = error_translation-error_previous_translation;
        error_previous_translation = error_translation;
        //ROS_INFO("error_derivation_translation: %f", error_derivation_translation);

        error_integral_translation += error_translation;
        //ROS_INFO("error_integral_translation: %f", error_integral_translation);

        //control of translation with a PID controller
        translation_speed = kpt * error_translation + kir * error_integral_translation + kdr * error_derivation_translation;
        ROS_INFO("translation_speed: %f", translation_speed);
    }
    else
        ROS_WARN("pid for translation will stop");

    bool obstacle_detected = ( fabs(closest_obstacle.x) < security_distance );

    if ( obstacle_detected )
    {
        ROS_WARN("obstacle detected: (%f, %f)", closest_obstacle.x, closest_obstacle.y);
        translation_speed = 0;
    }

}//perform_translation

void combine_rotation_and_translation()
{

    float coef_rotation = fabs(rotation_speed)/rotation_speed_max;
    if ( coef_rotation > 1 )
        coef_rotation = 1;
    float coef_translation = 1 - coef_rotation;

    translation_speed *= coef_translation;
    ROS_INFO("coef_rotation: %f, coef_translation: %f", coef_rotation, coef_translation);

    if ( translation_speed < 0 )
    {
        translation_speed = 0;
        ROS_WARN("translation_speed is negative");
    }

}//combine_rotation_and_translation

void move_robot()
{

    cond_goal = cond_rotation || cond_translation;

    geometry_msgs::Twist twist;
    twist.linear.x = translation_speed;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = rotation_speed;

    pub_cmd_vel.publish(twist);

}// move_robot

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;

    current_position = o->pose.pose.position;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach = *g;

}

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs) {

    init_obstacle = true;
    closest_obstacle = *obs;

}//closest_obstacleCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "action");

    ROS_INFO("(action_node) waiting for a /goal_to_reach");
    action bsObject;

    ros::spin();

    return 0;
}
