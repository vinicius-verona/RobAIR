// Signal handling
#include <signal.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_srvs/Empty.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#define translation_speed_max 0.9// in m/s
#define rotation_speed_max (M_PI/3) // 30 degres

float error_translation_threshold = 0.3;// in m

float kpt = 1;
float kit = 0;
float kdt = 0;

float error_rotation_threshold = M_PI/9;// in radians = 20 degres

float kpr = 0.5;
float kir = 0;
float kdr = 0;

using namespace std;

class advanced_action_node
{
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

    //pid for rotation
    float rotation_to_do, rotation_done;
    bool cond_rotation, only_rotation;
    float initial_orientation;// to store the initial orientation ie, before starting the pid for rotation control
    float current_orientation;// to store the current orientation: this information is provided by the odometer
    float error_rotation;
    float error_integral_rotation;
    float error_previous_rotation;
    float current_rotation_speed;

    //pid for translation
    float translation_to_do, translation_done;
    bool cond_translation;//boolean to check if we still have to translate or not
    geometry_msgs::Point initial_position;// to store the initial position ie, before starting the pid for translation control
    geometry_msgs::Point current_position;// to store the current position: this information is provided by the odometer
    float error_translation;
    float error_integral_translation;
    float error_previous_translation;
    float current_translation_speed;

    bool init_odom;
    bool init_obstacle;
    bool obstacle_detected;
    geometry_msgs::Point closest_obstacle;

public:

advanced_action_node()
{

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &advanced_action_node::odomCallback, this);

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &advanced_action_node::closest_obstacleCallback, this);

    // communication with datmo
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &advanced_action_node::goal_to_reachCallback, this);

    new_goal_to_reach = false;
    init_odom = false;
    init_obstacle = false;

    cond_translation = false;
    current_rotation_speed = 0;
    current_translation_speed = 0;
    cond_rotation = false;
    cond_translation = false;

    ros::Rate r(10);//this node is updated at 10hz

    while (ros::ok())
    {
        ros::spinOnce();
        update();
        r.sleep();
    }

}//advanced_action_node

void update()
{

    if ( init_odom && init_obstacle )
    { //wait for the initialization of odometer and detect_obstacle_node

        // we receive a new /goal_to_reach
        if ( new_goal_to_reach )        
            init_action();

        //we are performing a rotation and a translation
        if ( cond_translation )
        {            
            ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);
            compute_translation();
            compute_rotation();
            combine_rotation_and_translation();
            move_robot();
        }

   /*     if ( only_rotation && cond_rotation )
        {
            ROS_INFO("processing the /rotation_to_do received at %f", rotation_to_do*180/M_PI);
            compute_rotation();
            move_robot();
        }*/

    }
    else
    {
        if ( !init_odom )
            ROS_INFO("waiting for odometer");
        if ( !init_obstacle )
            ROS_INFO("waiting for obstacle_detection_node");
    }

}// update

void init_action()
{
    new_goal_to_reach = false;
    only_rotation = false;
    ROS_WARN("new goal received");
    ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

    cond_translation = translation_to_do > error_translation_threshold;
    if ( cond_translation )
    {

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
        ROS_WARN("translation_to_do is too low");

}// init_action

void compute_rotation()
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

    error_rotation = rotation_to_do-rotation_done;
    if ( error_rotation > M_PI )
    {
        ROS_WARN("error_rotation_done > 180 degrees: %f degrees -> %f degrees", error_rotation*180/M_PI, (error_rotation-2*M_PI)*180/M_PI);
        error_rotation -= 2*M_PI;
    }
    else
        if ( error_rotation < -M_PI )
        {
            ROS_WARN("error_rotation < -180 degrees: %f degrees -> %f degrees", error_rotation*180/M_PI, (error_rotation+2*M_PI)*180/M_PI);
            error_rotation += 2*M_PI;
        }
    ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f", rotation_to_do*180/M_PI, rotation_done*180/M_PI, error_rotation*180/M_PI);

/*    if ( error_translation>1 )
        cond_rotation = ( error_translation*fabs(error_rotation) > error_rotation_threshold );
    else
        cond_rotation = ( fabs(error_rotation) > error_rotation_threshold );*/
    cond_rotation = ( error_translation>1 ) || ( fabs(error_rotation) > error_rotation_threshold );
    cond_rotation = cond_rotation || ( only_rotation && fabs(error_rotation) > error_rotation_threshold );

    current_rotation_speed = 0;
    if ( cond_rotation )
    {
        //Implementation of a PID controller for rotation_to_do;
        float error_derivation_rotation = error_rotation-error_previous_rotation;
        error_previous_rotation = error_rotation;
        //ROS_INFO("error_derivation_rotation: %f", error_derivation_rotation);

        error_integral_rotation += error_rotation;
        //ROS_INFO("error_integral_rotation: %f", error_integral_rotation);

        //control of rotation with a PID controller
        current_rotation_speed = kpr * error_rotation + kir * error_integral_rotation + kdr * error_derivation_rotation;
        if ( current_rotation_speed > rotation_speed_max )
            current_rotation_speed = rotation_speed_max;
        if ( current_rotation_speed < -rotation_speed_max )
            current_rotation_speed = -rotation_speed_max;
        ROS_INFO("rotation_speed: %f degres/s = %f rad/s", current_rotation_speed*180/M_PI, current_rotation_speed);
    }
    else
        ROS_WARN("pid for rotation will stop");

}//compute_rotation

void compute_translation()
{

    ROS_INFO("current_position: (%f, %f), initial_position: (%f, %f)", current_position.x, current_position.y, initial_position.x, initial_position.y);
    translation_done = distancePoints( initial_position, current_position );
    error_translation = translation_to_do - translation_done;

    ROS_INFO("translation_to_do: %f, translation_done: %f, error_translation: %f", translation_to_do, translation_done, error_translation);

    cond_translation = ( fabs(error_translation) > error_translation_threshold );

    if ( fabs(closest_obstacle.x) < fabs(error_translation) )
    {
        error_translation = closest_obstacle.x;
        ROS_WARN("obstacle detected: (%f, %f)", closest_obstacle.x, closest_obstacle.y);
    }

    bool cond_obstacle = ( fabs(error_translation) > error_translation_threshold );

    current_translation_speed = 0;
    if ( cond_obstacle && cond_translation )
    {

        //Implementation of a PID controller for translation_to_do;
        float error_derivation_translation = error_translation-error_previous_translation;
        error_previous_translation = error_translation;
        //ROS_INFO("error_derivation_translation: %f", error_derivation_translation);

        error_integral_translation += error_translation;
        //ROS_INFO("error_integral_translation: %f", error_integral_translation);

        if ( error_translation < 1 )
            kpt=0.75;
        else
            kpt=1;

        //control of translation with a PID controller
        current_translation_speed = kpt * error_translation + kir * error_integral_translation + kdr * error_derivation_translation;
        if ( current_translation_speed > translation_speed_max )
            current_translation_speed = translation_speed_max;
        ROS_INFO("current_translation_speed: %f", current_translation_speed);
    }
    else
        ROS_WARN("pid for translation will stop");

}// compute_translation

void combine_rotation_and_translation()
{

    float coef_rotation = fabs(error_rotation);
    if ( coef_rotation > rotation_speed_max )
        coef_rotation = rotation_speed_max;
//    if ( coef_rotation < rotation_acceleration )
//        coef_rotation = 0;
    coef_rotation /= rotation_speed_max;
    float coef_translation = 1 - coef_rotation;

    current_translation_speed *= coef_translation;
    if ( current_translation_speed < 0 )
    {
        current_translation_speed = 0;
        ROS_WARN("current_translation_speed is negative");
    }

    ROS_INFO("coef_rotation: %f, coef_translation: %f\n", coef_rotation, coef_translation);
    ROS_INFO("current_translation_speed: %f", current_translation_speed);

}//combine_rotation_and_translation

void move_robot()
{

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;

    if ( cond_translation )
    {
        twist.linear.x = current_translation_speed;
        twist.angular.z = current_rotation_speed;
    }

    if ( only_rotation && cond_rotation )
        twist.angular.z = current_rotation_speed;

    pub_cmd_vel.publish(twist);

}// move_robot

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o)
{

    init_odom = true;

    current_position = o->pose.pose.position;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g)
{
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach = *g;
//	goal_to_reach.y *=5;

}

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs)
{

    init_obstacle = true;
    closest_obstacle = *obs;

}//closest_obstacleCallback

float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
// Distance between two points

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "action");
    ros::NodeHandle n;

    /*if ( ros::param::get("/action_node/locked", obstacle_detection) )
        if ( locked )
            ROS_INFO("locked: true");
        else
            ROS_INFO("locked: false");*/

    /*if ( ros::param::get("/action_node/obstacle_detection", obstacle_detection) )
        if ( obstacle_detection )
            ROS_INFO("obstacle_detection: true");
        else
            ROS_INFO("obstacle_detection: false");

    if ( ros::param::get("/action_node/translation_speed_max", translation_speed_max) )
        ROS_INFO("translation_speed_max: %f", translation_speed_max);

    if ( ros::param::get("/action_node/translation_acceleration", translation_acceleration) )
        ROS_INFO("translation_acceleration: %f", translation_acceleration);

    if ( ros::param::get("/action_node/translation_deceleration", translation_deceleration) )
        ROS_INFO("translation_deceleration: %f", translation_deceleration);

    if ( ros::param::get("/action_node/rotation_acceleration", rotation_acceleration) )
        ROS_INFO("rotation_acceleration: %f", rotation_acceleration);

    if ( ros::param::get("/action_node/rotation_deceleration", rotation_deceleration) )
        ROS_INFO("rotation_decceleration: %f", rotation_deceleration);*/

    if ( ros::param::get("/action_node/error_translation_threshold", error_translation_threshold) )
        ROS_INFO("error_translation_threshold: %f", error_translation_threshold);

    if ( ros::param::get("/action_node/error_rotation_threshold", error_rotation_threshold) )
        ROS_INFO("error_rotation_threshold: %f", error_rotation_threshold);

    if ( ros::param::get("/action_node/kpt", kpt) )
        ROS_INFO("kpt: %f", kpt);

    if ( ros::param::get("/action_node/kit", kit) )
        ROS_INFO("kit: %f", kit);

    if ( ros::param::get("/action_node/kdt", kdt) )
        ROS_INFO("kdt: %f", kdt);

    ROS_INFO("(action_node) waiting for a new /goal_to_reach");

    advanced_action_node bsObject;
    ros::spin();

    return 0;

}


