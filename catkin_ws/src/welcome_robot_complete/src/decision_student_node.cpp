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

#define waiting_for_a_person 0
#define rotating_to_the_person 1
#define moving_to_the_person 2
#define interacting_with_the_person 3
#define returning_to_the_base 4
#define resetting_orientation 5

//hall de de l'UFR
//#define base_x 25.5
//#define base_y -17
//#define base_orientation 12

//F218
/*#define base_position_x 6.5
#define base_position_y 4.3
#define base_orientation M_PI*/

//Maison
//#define base_position_x 0.25 //grande carte
#define base_position_x -.3
#define base_position_y -.2
#define base_orientation 0

//#define base_position_x 0
//#define base_position_y 0

//couloir
//#define base_x 7.4
//#define base_y -3.5

#define frequency_expected 25
#define max_base_distance 6

class decision_node {
private:

    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, person_tracked;
    geometry_msgs::Point person_position;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    float rotation_to_person;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    float translation_to_person;

    // communication with amcl or my localization
    ros::Subscriber sub_amcl;
    ros::Subscriber sub_localization;
    bool new_localization;
    bool init_localization;
    geometry_msgs::Point current_position;
    float current_orientation;
    float translation_to_base;
    float rotation_to_base;
    geometry_msgs::Point local_base_position;

    int current_state, previous_state;
    int frequency;
    geometry_msgs::Point base_position;
    geometry_msgs::Point origin_position;
    bool state_has_changed;
    /*int frequency_base1;
    int frequency_base2;*/

public:

decision_node() {

    // communication with datmo_node
    sub_person_position = n.subscribe("person_position", 1, &decision_node::person_positionCallback, this);

    // communication with rotation_node
    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);

    // communication with action_node
    pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the position of the person

    // communication with robot_moving_node
    sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

    // communication with amcl
    sub_amcl = n.subscribe("amcl_pose", 1, &decision_node::amclCallback, this);
    new_localization = false;
    init_localization = false;

    // communication with my localization node
    sub_localization = n.subscribe("localization", 1, &decision_node::localizationCallback, this);

    current_state = waiting_for_a_person;
    //current_state = returning_to_the_base;
    previous_state = -1;

    new_person_position = false;
    state_has_changed = false;

    base_position.x = base_position_x;
    base_position.y = base_position_y;

    origin_position.x = 0;
    origin_position.y = 0;

    person_tracked = false;

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

    //init_localization = true;
    if ( init_localization )
    {

        //state_has_changed = true;
  //      reset_frequency();
        update_variables();
 /*   switch ( current_state )
    {
        case waiting_for_a_person:
            process_waiting_for_a_person();

        case rotating_to_the_person:
            process_rotating_to_the_person();

        case moving_to_the_person:
            process_moving_to_the_person();

        case interacting_with_the_person:
            process_interacting_with_the_person();

        case returning_to_the_base:
            process_returning_to_the_base();

        case resetting_orientation:
        process_resetting_orientation();
    }*/

    if ( current_state == waiting_for_a_person )
        process_waiting_for_a_person();
    else
        if ( current_state == rotating_to_the_person )
            process_rotating_to_the_person();
        else
            if ( current_state == moving_to_the_person )
                process_moving_to_the_person();
            else
                if ( current_state == interacting_with_the_person )
                    process_interacting_with_the_person();
                else
                    if ( current_state == returning_to_the_base )
                        process_returning_to_the_base();
                    else
                        if ( current_state == resetting_orientation )
                            process_resetting_orientation();

    new_localization = false;
    new_person_position = false;

    state_has_changed = current_state != previous_state;
    previous_state = current_state;

    }
    else
        ROS_WARN("Initialize localization");

}// update

void update_variables()
{

    if ( new_person_position )
    {
        translation_to_person = distancePoints(origin_position, person_position);

        if ( translation_to_person > 0 )
        {
            rotation_to_person = acos( person_position.x / translation_to_person );
            if ( person_position.y < 0 )
                rotation_to_person *=-1;
        }
        else
            rotation_to_person = 0;
        person_tracked = person_position.x != 0 || person_position.y != 0;
    }        

    if ( new_localization )
    {
        local_base_position.x = base_position.x - current_position.x;
        local_base_position.y = base_position.y - current_position.y;

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        translation_to_base = distancePoints(origin_position, local_base_position);

        if ( translation_to_base > 0 )
        {

            //we compute the /rotation_to_do
            rotation_to_base = acos( local_base_position.x / translation_to_base );

            if ( local_base_position.y < 0 )
                rotation_to_base *=-1;

            ROS_INFO("rotation_to_base: %f", rotation_to_base*180/M_PI);
            rotation_to_base -= current_orientation;
            ROS_INFO("rotation_to_base: %f", rotation_to_base*180/M_PI);

            if ( rotation_to_base > M_PI )
            {
                ROS_WARN("rotation_to_base > 180 degrees: %f degrees -> %f degrees", rotation_to_base*180/M_PI, (rotation_to_base-2*M_PI)*180/M_PI);
                rotation_to_base -= 2*M_PI;
            }
            else
                if ( rotation_to_base < -M_PI )
                {
                    ROS_WARN("rotation_to_base < -180 degrees: %f degrees -> %f degrees", rotation_to_base*180/M_PI, (rotation_to_base+2*M_PI)*180/M_PI);
                    rotation_to_base += 2*M_PI;
                }

            local_base_position.x = translation_to_base*cos(rotation_to_base);
            local_base_position.y = translation_to_base*sin(rotation_to_base);

//                pub_person_position.publish(local_person_position);
        }
    }

}

void process_waiting_for_a_person()
{

    if ( state_has_changed )
        ROS_INFO("current_state: waiting_for_a_person");
    //exit(1);

    if ( new_person_position )
        current_state = rotating_to_the_person;

}

void process_rotating_to_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: rotating_to_the_person");
        frequency = 0;
    }

    //Processing of the state
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("rotation_to_person: %f degrees", rotation_to_person*180/M_PI);

        //send the rotation to do
        std_msgs::Float32 msg_rotation_to_person;
        msg_rotation_to_person.data = rotation_to_person;
        pub_rotation_to_do.publish(msg_rotation_to_person);
    }

    //1st condition: leave to state "moving_to_the_person"
    int old_frequency = frequency;
    if ( !robot_moving && fabs(rotation_to_person) < M_PI/9 )
    {
        frequency++;
        if ( frequency >= frequency )
        {
            current_state = moving_to_the_person;
            std_msgs::Float32 msg_rotation_to_person;
            msg_rotation_to_person.data = 0;
            pub_rotation_to_do.publish(msg_rotation_to_person);
        }
        else
            frequency = 0;
    }
    if ( old_frequency != frequency )
        ROS_INFO("frequency_rotation: %i\n", frequency);

    //2nd condition: leave to state "resetting_orientation"
    if ( !person_tracked )
    {
        current_state = resetting_orientation;
        std_msgs::Float32 msg_rotation_to_person;
        msg_rotation_to_person.data = 0;
        pub_rotation_to_do.publish(msg_rotation_to_person);
    }

}

void process_moving_to_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: moving_to_the_person");
        frequency = 0;
    }

    //processing new position for the person
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("translation_to_person: %f meters", translation_to_person);

        //send the goal to reach
        pub_goal_to_reach.publish(person_position);
    }

     //1st condition to leave the state
    int old_frequency = frequency;
    if ( ( !robot_moving ) && ( translation_to_person <= 1 ) )
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = interacting_with_the_person;
            pub_goal_to_reach.publish(origin_position);
        }
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
        ROS_INFO("frequency_moving: %i\n", frequency);

     //2nd condition to leave the state
     if ( !person_tracked || ( translation_to_base >= max_base_distance ) )
    {
         current_state = returning_to_the_base;
         pub_goal_to_reach.publish(origin_position);
    }

}

void process_interacting_with_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: interacting_with_the_person");
        frequency = 0;
    }

    //1st condition to leave the state
    int old_frequency = frequency;
    if ( translation_to_person>1 || !person_tracked )
    {
        frequency++;
        if ( frequency >= 50 )
            current_state = returning_to_the_base;
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
        ROS_INFO("frequency_interacting: %i\n", frequency);

}

void process_returning_to_the_base()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: returning_to_the_base");
        frequency = 0;
    }

    if ( new_localization || state_has_changed )
    {

        ROS_INFO("current_position: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        ROS_INFO("base_position: (%f, %f)", base_position.x, base_position.y);

        ROS_INFO("rotation_to_base: %f = %f - %f", rotation_to_base*180/M_PI, (rotation_to_base+current_orientation)*180/M_PI, current_orientation*180/M_PI);
        ROS_INFO("local_base_position: (r = %f, theta = %f) -> (x = %f, y = %f)", translation_to_base, rotation_to_base*180/M_PI, local_base_position.x, local_base_position.y);
/*        pub_goal_to_reach.publish(origin_position);
        ROS_INFO("press enter to continue");
        getchar();*/

        //send the goal to reach
        //person_position = local_person_position;
        pub_goal_to_reach.publish(local_base_position);
    }

    //1st condition to leave
    int old_frequency = frequency;
    if ( !robot_moving && translation_to_base<=1 )
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = resetting_orientation;
            pub_goal_to_reach.publish(origin_position);
        }
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
         ROS_INFO("frequency_base: %i\n", frequency);

}

void process_resetting_orientation()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: initializing_rotation");
        frequency = 0;
    }

    if ( new_localization || state_has_changed )
    {
        ROS_INFO("current_position: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        ROS_INFO("base_orientation: %f", base_orientation*180/M_PI);

    /*    if ( fabs(base_orientation-current_orientation) > M_PI/9 )
        {*/
            //send the rotation to do
            std_msgs::Float32 msg_rotation_to_do;

            /*msg_rotation_to_do.data = 0;
            pub_rotation_to_do.publish(msg_rotation_to_do);

            ROS_INFO("press enter to continue");
            getchar();*/

            //send the rotation to do
            msg_rotation_to_do.data = base_orientation-current_orientation;
            pub_rotation_to_do.publish(msg_rotation_to_do);
            state_has_changed = false;
        //}
    }

    //1st condition to leave
    int old_frequency = frequency;
    if ( !robot_moving && fabs(base_orientation-current_orientation) < M_PI/6)
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = waiting_for_a_person;
            state_has_changed = true;
        }
    }
    else
        frequency = 0;

    if ( frequency != old_frequency )
        ROS_INFO("frequency_init: %i\n", frequency);

/*        else
            ROS_WARN("localisation not initialized");*/

}

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void person_positionCallback(const geometry_msgs::Point::ConstPtr& g)
{
// process the goal received from moving_persons detector

    new_person_position = true;
    person_position.x = g->x;
    person_position.y = g->y;

}

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state)
{

    robot_moving = state->data;

}//robot_movingCallback

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& l)
{
// process the localization received from amcl_pose

    new_localization = true;
    init_localization = true;
    current_position = l->pose.pose.position;
    current_orientation = tf::getYaw(l->pose.pose.orientation);

}

void localizationCallback(const geometry_msgs::Point::ConstPtr& l)
{
// process the localization received from my localization

    new_localization = true;
    init_localization = true;
    current_position = *l;
    current_orientation = l->z;

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision");

    decision_node bsObject;

    ros::spin();

    return 0;
}
