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
#define observing_the_person 1
#define rotating_to_the_person 2
#define moving_to_the_person 3
#define interacting_with_the_person 4
#define rotating_to_the_base 5
#define moving_to_the_base 6
#define resetting_orientation 7

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

class decision_node
{
private:

    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, person_tracked, person_is_moving;
    geometry_msgs::Point person_position, last_person_position;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;
    geometry_msgs::Point closest_obstacle;
    bool init_obstacle;

    // communication with odom
    ros::Subscriber sub_odometry;
    geometry_msgs::Point base_position;
    geometry_msgs::Point current_position;
    float current_orientation;
    float translation_to_base;
    float rotation_to_base;
    bool init_odom;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    ros::Publisher pub_rotation_to_do;
    float translation_to_person;
    float rotation_to_person;

    // communication with aruco
    ros::Subscriber sub_aruco_position;
    bool new_aruco;
    geometry_msgs::Point aruco_position;

    int current_state, previous_state;
    int frequency;
    geometry_msgs::Point origin_position;
    bool state_has_changed;
    geometry_msgs::Point observed_person_position;

public:

decision_node()
{

    // communication with datmo_node
    sub_person_position = n.subscribe("person_position", 1, &decision_node::person_positionCallback, this);

    // communication with action_node
    pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the position of the person
    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 1);     // Preparing a topic to publish the rotation of the person

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &decision_node::odomCallback, this);

    // communication with robot_moving_node
    sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

    // communication with aruco_node
    sub_aruco_position = n.subscribe("aruco_position", 1, &decision_node::aruco_positionCallback, this);
    sub_aruco_position = n.subscribe("localization", 1, &decision_node::aruco_positionCallback, this);

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &decision_node::closest_obstacleCallback, this);

    current_state = waiting_for_a_person;
    //current_state = moving_to_the_base;
    previous_state = -1;

    new_person_position = false;
    state_has_changed = false;
    init_odom = false;
    init_obstacle = false;

    origin_position.x = 0;
    origin_position.y = 0;

    last_person_position.x = 0;
    last_person_position.y = 0;

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
    if ( init_odom /*&& init_obstacle*/ )
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

        case moving_to_the_base:
            process_moving_to_the_base();

        case resetting_orientation:
        process_resetting_orientation();
    }*/

  /*  if ( distancePoints(base_position, current_position) > 6 &&
         current_state != rotating_to_the_base && current_state != moving_to_the_base )
        current_state = rotating_to_the_base;*/

    if ( current_state == waiting_for_a_person )
        process_waiting_for_a_person();
    else
        if ( current_state == observing_the_person )
            process_observing_the_person();
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
                        if ( current_state == rotating_to_the_base )
                            process_rotating_to_the_base();
                        else
                            if ( current_state == moving_to_the_base )
                                process_moving_to_the_base();
                            else
                                if ( current_state == resetting_orientation )
                                    process_resetting_orientation();

    new_person_position = false;

    state_has_changed = current_state != previous_state;
    previous_state = current_state;

    }
    else
        ROS_WARN("Initialize odometry");

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

        person_is_moving = distancePoints(last_person_position, person_position ) > 0.1;
        if ( person_is_moving )
        {
            last_person_position = person_position;
         //   ROS_INFO("person is moving");
        }
//        else
  //          ROS_INFO("person is not moving");
    }        

    // in the frame of odom
    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    geometry_msgs::Point local_base_position;
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

       // ROS_INFO("rotation_to_base: %f", rotation_to_base*180/M_PI);
        rotation_to_base -= current_orientation;
        //ROS_INFO("rotation_to_base: %f", rotation_to_base*180/M_PI);

        if ( rotation_to_base > M_PI )
        {
           // ROS_WARN("rotation_to_base > 180 degrees: %f degrees -> %f degrees", rotation_to_base*180/M_PI, (rotation_to_base-2*M_PI)*180/M_PI);
            rotation_to_base -= 2*M_PI;
        }
        else
            if ( rotation_to_base < -M_PI )
            {
             //   ROS_WARN("rotation_to_base < -180 degrees: %f degrees -> %f degrees", rotation_to_base*180/M_PI, (rotation_to_base+2*M_PI)*180/M_PI);
                rotation_to_base += 2*M_PI;
            }
    }

}

void process_waiting_for_a_person()
{

/*    if ( state_has_changed )
    {*/
        ROS_INFO("current_state: waiting_for_a_person");
        //getchar();
    /*}
    base_position = current_position;*/

    if ( new_person_position )
        current_state = observing_the_person;

}

void process_observing_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: observing_the_person");

//        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
//        observed_person_position = person_position;
        //getchar();
    }

    //Processing of the state
    if ( new_person_position )
    {
        ROS_INFO("current_state: observing_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);

        //1st condition: leave to state "moving_to_the_person"
        int old_frequency = frequency;
        bool cond = ( distancePoints(observed_person_position, person_position) <= 0.1 );
        //ROS_INFO("cond: %i\n", cond);

//        if ( cond )
        if ( !person_is_moving )
        {
            frequency++;
            if ( frequency >= frequency_expected )
            {
                current_state = rotating_to_the_person;
               // current_state = moving_to_the_person;
                /*std_msgs::Float32 msg_rotation_to_person;
                msg_rotation_to_person.data = 0;
                pub_rotation_to_do.publish(msg_rotation_to_person);*/
            }
               ROS_INFO("person is not moving");
        }
        else
        {
            frequency = 0;
            ROS_INFO("person is moving");
        //    observed_person_position = person_position;
        }

      //  if ( old_frequency != frequency )
            ROS_INFO("frequency: %i\n", frequency);
    }

    //2nd condition: leave to state "resetting_orientation"
    if ( !person_tracked )
    {
        current_state = waiting_for_a_person;
    /*    geometry_msgs::Float32 msg_rotation_to_person;
        msg_rotation_to_person.data = 0;
        pub_rotation_to_do.publish(msg_rotation_to_person);*/
    }

}

void process_rotating_to_the_person()
{

    if ( state_has_changed )
    {
        frequency = 0;
        //getchar();
    }

    //Processing of the state
    if ( new_person_position )
    {
        ROS_INFO("current_state: rotating_to_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("rotation_to_person: %f degrees", rotation_to_person*180/M_PI);

    //    if ( fabs(rotation_to_person) > M_PI/9 )
      //      {//send the rotation to do*/
            std_msgs::Float32 msg_rotation_to_do;
            msg_rotation_to_do.data = rotation_to_person;
            pub_rotation_to_do.publish(msg_rotation_to_do);
        //}


    //1st condition: leave to state "moving_to_the_person"
    int old_frequency = frequency;
  //  bool cond = !robot_moving && fabs(rotation_to_person) < M_PI/9;
    //ROS_INFO("cond: %i\n", cond);
    //cond = cond && ( fabs(rotation_to_person) < M_PI/12 );
    //ROS_INFO("cond: %i\n", cond);

    //if ( cond )
    if ( !person_is_moving && fabs(rotation_to_person) < M_PI/18 )
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = moving_to_the_person;
            /*std_msgs::Float32 msg_rotation_to_person;
            msg_rotation_to_person.data = 0;
            pub_rotation_to_do.publish(msg_rotation_to_person);*/
        }
        ROS_INFO("person is not moving");
    }
    else
    {
        frequency = 0;
        ROS_INFO("person is moving");
    }

    //if ( old_frequency != frequency )
    //{
      //  ROS_INFO("current_state: rotating_to_the_person");
        ROS_INFO("frequency_rotation: %i\n", frequency);
    }

    //2nd condition: leave to state "resetting_orientation"
    if ( !person_tracked )
    {
        current_state = resetting_orientation;
    /*    geometry_msgs::Float32 msg_rotation_to_person;
        msg_rotation_to_person.data = 0;
        pub_rotation_to_do.publish(msg_rotation_to_person);*/
    }

}

void process_moving_to_the_person()
{

    if ( state_has_changed )
    {
        frequency = 0;
//        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
//        ROS_INFO("translation_to_person: %f meters, rotation_to_person: %f degrees", translation_to_person, rotation_to_person*180/M_PI);
     //   getchar();
    }

    //processing new position for the person
    if ( new_person_position )
    {
        ROS_INFO("current_state: moving_to_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("translation_to_person: %f meters, rotation_to_person: %f degrees", translation_to_person, rotation_to_person*180/M_PI);

        //send the goal to reach

     //   if ( translation_to_person>0.5 )
       // {
            geometry_msgs::Point msg_goal_to_reach;
            msg_goal_to_reach.x = person_position.x;
            msg_goal_to_reach.y = person_position.y;
            pub_goal_to_reach.publish(person_position);
        //}

     //1st condition to leave the state
    int old_frequency = frequency;
    //if ( ( !robot_moving ) && ( translation_to_person <= 0.5 ) )
    if ( !person_is_moving && translation_to_person<0.5 )
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = interacting_with_the_person;
            //pub_goal_to_reach.publish(origin_position);
        }
        ROS_INFO("person is not moving");

    }
    else
    {
        frequency = 0;
        ROS_INFO("person is not moving");
    }

    //if ( old_frequency != frequency )
        ROS_INFO("frequency_moving: %i\n", frequency);
    }

     //2nd condition to leave the state
     if ( !person_tracked /*|| ( translation_to_base >= max_base_distance )*/ )
    {
         current_state = rotating_to_the_base;
         //pub_goal_to_reach.publish(origin_position);
    }

}

void process_interacting_with_the_person()
{

    ROS_INFO("current_state: interacting_with_the_person");
    if ( state_has_changed )
    {
        frequency = 0;
        //getchar();
    }

    if ( new_person_position )
    {
        //1st condition to leave the state
        int old_frequency = frequency;
        if ( translation_to_person>1 || !person_tracked )
        {
            frequency++;
            if ( frequency >= 50 )
                current_state = rotating_to_the_base;
        }
        else
            frequency = 0;

        //if ( old_frequency != frequency )
            ROS_INFO("frequency_interacting: %i\n", frequency);
    }
    if ( !person_tracked )
        current_state = rotating_to_the_base;

}

void process_rotating_to_the_base()
{

    ROS_INFO("current_state: rotating_to_the_base");
    if ( state_has_changed )
    {
        frequency = 0;
        //getchar();
        std_msgs::Float32 msg_rotation_to_do;
        msg_rotation_to_do.data = rotation_to_base;
        pub_rotation_to_do.publish(msg_rotation_to_do);
    }

    //1st condition: leave to state "rotating_to_the_base"
    int old_frequency = frequency;
    bool cond = !robot_moving;

    if ( cond )
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = moving_to_the_base;
            /*std_msgs::Float32 msg_rotation_to_person;
            msg_rotation_to_person.data = 0;
            pub_rotation_to_do.publish(msg_rotation_to_person);*/
        }
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
        ROS_INFO("frequency_rotation: %i\n", frequency);
}

void process_moving_to_the_base()
{

    ROS_INFO("current_state: moving_to_the_base");
    if ( state_has_changed )
    {
        frequency = 0;
        exit(1);
    }

    if ( new_aruco )
    {
        ROS_INFO("aruco_position: (%f, %f)", aruco_position.x, aruco_position.y);

        //send the goal to reach
        geometry_msgs::Point msg_goal_to_reach;
        msg_goal_to_reach = aruco_position;
        pub_goal_to_reach.publish(msg_goal_to_reach);

        //1st condition to leave
        int old_frequency = frequency;
        if ( !robot_moving && aruco_position.x<=0.5 )
        {
            frequency++;
            if ( frequency >= frequency_expected )
            {
                current_state = resetting_orientation;
                //  pub_goal_to_reach.publish(origin_position);
            }
        }
        else
            frequency = 0;

        if ( old_frequency != frequency )
            ROS_INFO("frequency_base: %i\n", frequency);
    }

}

void process_resetting_orientation()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: resetting_orientation");
        frequency = 0;
//        exit(1);
    }

    //1st condition to leave
    int old_frequency = frequency;
    if ( !robot_moving )
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

void odomCallback(const nav_msgs::Odometry::ConstPtr& o)
{

    init_odom = true;

    current_position = o->pose.pose.position;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state)
{

    robot_moving = state->data;

}//robot_movingCallback

void aruco_positionCallback(const std_msgs::Float32::ConstPtr& y)
{
    new_aruco = true;
    aruco_position.x = closest_obstacle.x;
    aruco_position.y = y->data;

}//robot_movingCallback

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs)
{

    init_obstacle = true;
    closest_obstacle = *obs;

}//closest_obstacleCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision_node");

    decision_node bsObject;

    ros::spin();

    return 0;
}
