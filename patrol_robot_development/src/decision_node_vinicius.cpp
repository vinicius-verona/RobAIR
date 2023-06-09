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
#include <iostream>


#define waiting_for_a_person 0
#define observing_the_person 1
#define rotating_to_the_person 2
#define moving_to_the_person 3
#define interacting_with_the_person 4
#define rotating_to_the_base 5
#define returning_to_the_base 6
#define resetting_orientation 7

#define DEBUG_GETCHAR_ENABLED 1

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
/*#define base_position_x -.3
#define base_position_y -.2
#define base_orientation 0*/

//#define base_position_x 0
//#define base_position_y 0

//couloir
//#define base_x 7.4
//#define base_y -3.5

//Philip Plateau Test
#define base_position_x 0
#define base_position_y 0



#define frequency_expected 25
#define MAX_BASE_DIST 5.0

#define ARUCO_MODE 1 //if 0, use only odometry

#define AUDIO_COOLDOWN 30 //number of node cycles to wait before saying the same thing again

class decision_node
{
private:

    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, person_tracked;
    geometry_msgs::Point person_position;

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
    bool new_aruco, no_aruco;
    geometry_msgs::Point aruco_position;

    int current_state, previous_state;
    int frequency;
    geometry_msgs::Point origin_position;
    bool state_has_changed;
    geometry_msgs::Point observed_person_position;

    //Display and sound
    //communication with audio_output node
    ros::Publisher pub_play_sound_file;
    //communication with display_output node
    ros::Publisher pub_display_image_file;

    //force odom reset via this topic
    ros::Publisher pub_change_odom;

    //Audio
    std::map<int, std::string> sound_file_paths;
    //Display
    std::map<int, std::string> display_file_paths;
    const std::string image_dir = "~/RobAIR/catkin_ws/src/welcome_robot_complete/resources/robot_images/";
    const std::string sound_dir = "~/RobAIR/catkin_ws/src/welcome_robot_complete/resources/robot_sounds/";

    float m_max_base_distance, rot_sign_aruco_search;
    int bonjour_timer = 0;
    float base_orientation = 0;


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
    sub_aruco_position = n.subscribe("robair_goal", 1, &decision_node::aruco_positionCallback, this);

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &decision_node::closest_obstacleCallback, this);

    pub_play_sound_file = n.advertise<std_msgs::String>("play_sound_file", 1);
    pub_display_image_file = n.advertise<std_msgs::String>("display_image_file", 1);

    pub_change_odom = n.advertise<geometry_msgs::Point>("change_odometry", 1);

    current_state = waiting_for_a_person;
    previous_state = -1;

    new_person_position = false;
    new_aruco = false;
    state_has_changed = false;
    init_odom = false;
    init_obstacle = false;

    origin_position.x = 0;
    origin_position.y = 0;
    origin_position.z = 0; // used when passing to reset odom, encodes orientation around z axis in radians.

    person_tracked = false;


    //Map sounds and images to states
    sound_file_paths[observing_the_person] =  sound_dir + "0-Bonjour_court_these_philip.wav";
    display_file_paths[observing_the_person] = image_dir + "bonjour.png";

    /*sound_file_paths[rotating_to_the_person] =  sound_dir + "0-Bonjour_court_these_philip.wav";
    display_file_paths[rotating_to_the_person] = image_dir + "bonjour.png";*/

    display_file_paths[moving_to_the_person] = image_dir + "flyer.png";

    sound_file_paths[interacting_with_the_person] = sound_dir + "3-HumHum1.wav";
    display_file_paths[interacting_with_the_person] = image_dir + "note.png";

    sound_file_paths[rotating_to_the_base] = sound_dir + "0-Aurevoir.wav";
    display_file_paths[rotating_to_the_base] = image_dir + "merci.png";

    display_file_paths[returning_to_the_base] = image_dir + "retourne_base.png";

    display_file_paths[resetting_orientation] = image_dir + "bonjour.png";

    m_max_base_distance = MAX_BASE_DIST;

    rot_sign_aruco_search = 1.0;
    aruco_position.x = 0.0; aruco_position.y = 0.0;

	
	ROS_INFO("vinicius node - press enter\n");
	getchar();


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
    if ( init_odom && init_obstacle )
    {
        //state_has_changed = true;
        //reset_frequency();

        update_variables();

        //Pass state information in the goal .z component for social nav planner to know if we're going towards a person.
        person_position.z = (float)current_state;

        /*
        switch ( current_state )
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
        }
        */

        ROS_INFO ("\n translation_to_base is %.3f m\n", translation_to_base);
        // Make sure the robot does not go too far from the base

        if ( distancePoints(base_position, current_position) > m_max_base_distance &&
             current_state != rotating_to_the_base &&
             current_state != returning_to_the_base) {
            //TODO: something like implement a new state called "stop_robot", which publishes 0,0 GTR, and waits for robot to stop moving, then transitions to RTB
            pub_goal_to_reach.publish(origin_position);
            sleep(3);
            current_state = rotating_to_the_base;
        }


        if ( current_state == waiting_for_a_person )
            process_waiting_for_a_person();
        else if ( current_state == observing_the_person )
            process_observing_the_person();
        else if ( current_state == rotating_to_the_person )
            process_rotating_to_the_person();
        else if ( current_state == moving_to_the_person )
            process_moving_to_the_person();
        else if ( current_state == interacting_with_the_person )
            process_interacting_with_the_person();
        // else if ( current_state == rotating_to_the_base )
        //     process_rotating_to_the_base();
        // else if ( current_state == returning_to_the_base )
        //     process_returning_to_the_base();
        // else if ( current_state == resetting_orientation )
        //     process_resetting_orientation();

        new_person_position = false;
        new_aruco = false;

        state_has_changed = current_state != previous_state;
        previous_state = current_state;

        //If we just changed state, send sound and image display request based on current state.
        if (state_has_changed) {
            ROS_WARN("State changed, checking sound and display.\n");
            if (!(current_state == observing_the_person && bonjour_timer > 0)) {
                display_and_sound(current_state);
                if (current_state == observing_the_person) {
                    bonjour_timer = AUDIO_COOLDOWN;
                }
            }
        }
        //decrement timer
        bonjour_timer = std::max(--bonjour_timer, 0);

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
    }        

    // in the frame of odom
    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    geometry_msgs::Point local_base_position; // Not really the real base position in RobAIR cartesian coordinates, just an intermediate step to compute rotation.
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

    if ( state_has_changed )
    {
        ROS_INFO("current_state: waiting_for_a_person");
        //reset odometry so that the translation_to_base calculation does not suffer from long-term drift.
        //assumption is that robair is near a base aruco marker when it enters this state, hence this is considered the new odom origin.
        pub_change_odom.publish(origin_position);
        base_orientation = current_orientation;
    }

    if ( new_person_position ) {
        current_state = observing_the_person;
        //if you want to automate rosbag, start recording here.
    }


}

void process_observing_the_person()
{

    ROS_INFO("current_state: observing_the_person");
    if ( state_has_changed )
    {
//        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
        observed_person_position = person_position;
        //getchar();
    }

    //Processing of the state
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)\n", person_position.x, person_position.y);

        //1st condition: leave to state "moving_to_the_person"
        int old_frequency = frequency;
        bool cond = ( distancePoints(observed_person_position, person_position) <= 0.5 );
        //ROS_INFO("cond: %i\n", cond);

        if ( cond )
        {
            frequency++;
            if ( frequency >= frequency_expected - 15 )
            {
                current_state = rotating_to_the_person;
                /*std_msgs::Float32 msg_rotation_to_person;
                msg_rotation_to_person.data = 0;
                pub_rotation_to_do.publish(msg_rotation_to_person);*/
            }
        }
        else
        {
            frequency = 0;
            observed_person_position = person_position;
        }

        if ( old_frequency != frequency )
            ROS_INFO("frequency_rotation: %i\n", frequency);
    }

    if ( !person_tracked )
    {
        current_state = waiting_for_a_person;
    }

}

void process_rotating_to_the_person()
{

    ROS_INFO("current_state: rotating_to_the_person");
    if ( state_has_changed )
    {
        frequency = 0;
        //getchar();
    }

    //Processing of the state
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("rotation_to_person: %f degrees\n", rotation_to_person*180/M_PI);

        if ( fabs(rotation_to_person) > M_PI/6 )
            {//send the rotation to do*/
            std_msgs::Float32 msg_rotation_to_do;
            msg_rotation_to_do.data = rotation_to_person;
            pub_rotation_to_do.publish(msg_rotation_to_do);
        }
    }

    //1st condition: leave to state "moving_to_the_person"
    int old_frequency = frequency;
    bool cond = !robot_moving;
    //ROS_INFO("cond: %i\n", cond);
    //cond = cond && ( fabs(rotation_to_person) < M_PI/12 );
    //ROS_INFO("cond: %i\n", cond);

    if ( cond )
    {
        frequency++;
        if ( frequency >= frequency_expected - 20 )
        {
            current_state = moving_to_the_person;
            ROS_INFO("Switching to move to person\n");
            // getchar();
           
            /*std_msgs::Float32 msg_rotation_to_person;
            msg_rotation_to_person.data = 0;
            pub_rotation_to_do.publish(msg_rotation_to_person);*/
        }
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
    {
        ROS_INFO("current_state: rotating_to_the_person");
        ROS_INFO("frequency_rotation: %i out of %d\n", frequency, frequency_expected - 10);
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

    ROS_INFO("current_state: moving_to_the_person");

    if (DEBUG_GETCHAR_ENABLED) getchar();

    if ( state_has_changed )
    {
        frequency = 0;
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("translation_to_person: %f meters, rotation_to_person: %f degrees", translation_to_person, rotation_to_person*180/M_PI);
     //   getchar();
    }

    //processing new position for the person
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        ROS_INFO("translation_to_person: %f meters, rotation_to_person: %f degrees\n", translation_to_person, rotation_to_person*180/M_PI);

        //send the goal to reach

        if ( translation_to_person>0.5 )
        {
            pub_goal_to_reach.publish(person_position);
            ROS_WARN("person-position-decision-node: %f, %f\n", person_position.x, person_position.y)
        }
    }

     //1st condition to leave the state
    int old_frequency = frequency;
    if ( ( !robot_moving ) && ( translation_to_person <= 1.0 ) )
    {
        frequency++;
        if ( frequency >= frequency_expected )
        {
            current_state = interacting_with_the_person;
        }
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
        ROS_INFO("frequency_moving: %i\n", frequency);

     //2nd condition to leave the state
     if ( !person_tracked || ( translation_to_base >= m_max_base_distance ) )
    {
         if (!person_tracked) {
            ROS_INFO("Got person_position (0, 0): Person Lost\n");
         }
         else {
             ROS_INFO("Too far from base\n");
         }

         if (DEBUG_GETCHAR_ENABLED) getchar();

         current_state = rotating_to_the_base;
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

    if (DEBUG_GETCHAR_ENABLED) getchar();
    pub_goal_to_reach.publish(origin_position);

    //1st condition to leave the state
    int old_frequency = frequency;
    if ( translation_to_person>1 || !person_tracked )
    {
        frequency++;
        if ( frequency >= 30 )
            current_state = rotating_to_the_base;
    }
    else
        frequency = 0;

    if ( old_frequency != frequency )
        ROS_INFO("frequency_interacting: %i\n", frequency);

}

void process_rotating_to_the_base()
{
    std_msgs::Float32 msg_rotation_to_do;
    ROS_INFO("current_state: rotating_to_the_base");
    if ( state_has_changed )
    {
        frequency = 0;
        //initialize rotation to do based on odometry
        msg_rotation_to_do.data = rotation_to_base;
        pub_rotation_to_do.publish(msg_rotation_to_do);
        no_aruco = true;
        if (rotation_to_base > 0.0) {
            rot_sign_aruco_search = 1.0;
        }
        else {
            rot_sign_aruco_search = -1.0;
        }
    }

    //1st condition: leave to state "rotating_to_the_base"
    int old_frequency = frequency;
    bool cond = !robot_moving;

    if ( cond )
    {
        frequency++;
        if ( frequency >= (frequency_expected - 15))
        {
            if (no_aruco) {
                //didn't see any aruco since last rotation, rotate again
                std_msgs::Float32 rot;
                rot.data = (M_PI/2.0 + (rand() % 6) / 6.0) * rot_sign_aruco_search;
                pub_rotation_to_do.publish(rot);
                no_aruco = true;
                frequency = 0;
            }
            else {
                //we saw an aruco, and we're stopped: go to base
                current_state = returning_to_the_base;
            }
        }
    }
    else {
        frequency = 0;
    }

    if (new_aruco) {
        no_aruco = false;
        //we see the marker, use that as the orientation.
        float aruco_dist = distancePoints(origin_position, aruco_position);
        float aruco_rot = 0.0;
        if ( aruco_dist > 0 )
        {
            aruco_rot = acos( aruco_position.x / aruco_dist );
            if ( aruco_position.y < 0 )
                aruco_rot *=-1;
        }
        msg_rotation_to_do.data = aruco_rot;
        ROS_WARN("Aruco visible, correcting rotation\n");
        pub_rotation_to_do.publish(msg_rotation_to_do);
    }



    if ( old_frequency != frequency )
        ROS_INFO("frequency_rotation: %i\n", frequency);
}


void process_returning_to_the_base()
{

    ROS_INFO("current_state: returning_to_the_base");
    if ( state_has_changed )
    {
        //send the goal to reach
        geometry_msgs::Point msg_goal_to_reach;
        msg_goal_to_reach = aruco_position;
        msg_goal_to_reach.z = (float)current_state;
        pub_goal_to_reach.publish(msg_goal_to_reach);
        frequency = 0;
        no_aruco = true;

    }

    if ( new_aruco )
    {
        no_aruco = false;
        ROS_INFO("aruco_position: (%f, %f)", aruco_position.x, aruco_position.y);

        //send the goal to reach
        geometry_msgs::Point msg_goal_to_reach;
        msg_goal_to_reach = aruco_position;
        msg_goal_to_reach.z = (float)current_state;
        pub_goal_to_reach.publish(msg_goal_to_reach);
    }

    //1st condition to leave
    int old_frequency = frequency;
    if ( !robot_moving /*&& !no_aruco*/ && aruco_position.x <= 1.0 )
    {
        frequency++;
        if ( frequency >= frequency_expected - 15)
        {
            current_state = resetting_orientation;
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
        ROS_INFO("current_state: resetting_orientation");
        frequency = 0;
        std_msgs::Float32 rot;
        #if ARUCO_MODE
            if (previous_state == returning_to_the_base) {
                //assume robair will be facing the base upon arriving, so make it rotate 180°
                rot.data = M_PI;
                pub_rotation_to_do.publish(rot);
            }
            else {
                rot.data = base_orientation-current_orientation;
                pub_rotation_to_do.publish(rot);
            }
        #else
            rot.data = base_orientation-current_orientation;
            pub_rotation_to_do.publish(rot);
        #endif
    }

    //1st condition to leave
    int old_frequency = frequency;
    if ( !robot_moving )
    {
        frequency++;
        if ( frequency >= frequency_expected - 15)
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


//Sound and Display based on current state
void display_and_sound(int curr_state) {
    std_msgs::String s;

    //check if we have a sound corresponding to this state, and send message to audio_output node if we do
    if (sound_file_paths.find(curr_state) != sound_file_paths.end()) {
        s.data = sound_file_paths.at(curr_state).c_str();
        ROS_INFO("found sound for state %d", current_state);
        if (!s.data.empty()) {
            ROS_INFO("Publishing\n");
            pub_play_sound_file.publish(s);
        }
    }

    //check if we have an image corresponding to this state, and send message to display_output node if we do
    if (display_file_paths.find(curr_state) != display_file_paths.end()) {
        s.data = display_file_paths.at(curr_state).c_str();
        if (!s.data.empty()) {
            pub_display_image_file.publish(s);
        }
    }
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

void aruco_positionCallback(const geometry_msgs::Point aruco_msg)
{
    new_aruco = (aruco_msg.x > 0.1) && fabs(aruco_msg.y) > 0.01;
    aruco_position.x = aruco_msg.x;
    aruco_position.y = aruco_msg.y;

}//aruco_callback

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
    ros::init(argc, argv, "decision");

    decision_node bsObject;

    ros::spin();

    return 0;
}
