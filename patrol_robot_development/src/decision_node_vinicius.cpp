#include <geometry_msgs/Twist.h>
#include <patrol_robot_development/ObstacleAvoidanceMsg.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <iostream>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

// Patrol robot processes
#define searching_aruco_marker 0
#define moving_to_aruco_marker 1
#define avoiding_lateral_crash 2

// Future process
// #define avoiding_obstacle 3
// #define rotate_to_base 4
// #define return_to_base 5

#define safe_distance_from_aruco 1
#define lateral_safety_threshold 0.75

#define DEBUG_GETCHAR_ENABLED 0

// Philip Plateau Test
#define base_position_x 0
#define base_position_y 0

#define frequency_expected 25
#define MAX_BASE_DIST 5.0
#define min_angle 0.1  // 0.1 rad = 5 degrees

#define ARUCO_MODE 1   // if 0, use only odometry
#define AUDIO_COOLDOWN \
    30  // number of node cycles to wait before saying the same thing again

float clamp(float orientation) {
    while (orientation > M_PI)
        orientation -= 2 * M_PI;

    while (orientation < -M_PI)
        orientation += 2 * M_PI;

    return orientation;
}

class decision_node {
private:
    ros::NodeHandle n;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with obstacle_detection and obstacle_avoidance
    ros::Subscriber sub_obstacle_detection;
    ros::Subscriber sub_obstacle_avoidance;
    geometry_msgs::Point closest_obstacle;
    geometry_msgs::Point lt_closest_obstacle;
    geometry_msgs::Point rt_closest_obstacle;
    float lt_obstacle_distance;
    float rt_obstacle_distance;
    bool init_obstacle;
    bool init_lateral_obstacles;

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

    // communication with aruco
    ros::Subscriber sub_aruco_position;
    bool new_aruco, no_aruco;
    geometry_msgs::Point aruco_position;

    int current_state, previous_state;
    int frequency;
    geometry_msgs::Point origin_position;
    bool state_has_changed;

    // force odom reset via this topic
    ros::Publisher pub_change_odom;

    float m_max_base_distance, rot_sign_aruco_search;
    float base_orientation = 0;

public:
    decision_node() {
        // communication with action_node
        pub_goal_to_reach = n.advertise<geometry_msgs::Point>(
            "goal_to_reach",
            1);  // Preparing a topic to publish the position of the person
        pub_rotation_to_do = n.advertise<std_msgs::Float32>(
            "rotation_to_do",
            1);  // Preparing a topic to publish the rotation of the person

        // communication with odometry
        sub_odometry =
            n.subscribe("odom", 1, &decision_node::odomCallback, this);

        // communication with robot_moving_node
        sub_robot_moving = n.subscribe(
            "robot_moving", 1, &decision_node::robot_movingCallback, this);

        // communication with aruco_node
        sub_aruco_position = n.subscribe(
            "robair_goal", 1, &decision_node::aruco_positionCallback, this);

        // communication with obstacle_detection
        sub_obstacle_detection =
            n.subscribe("closest_obstacle", 1,
                        &decision_node::closest_obstacleCallback, this);

        // communication with obstacle_avoidance
        sub_obstacle_avoidance =
            n.subscribe("lateral_distances", 1,
                        &decision_node::lateral_distancesCallback, this);
        pub_change_odom =
            n.advertise<geometry_msgs::Point>("change_odometry", 1);

        current_state  = searching_aruco_marker;
        previous_state = -1;

        new_aruco              = false;
        state_has_changed      = false;
        init_odom              = false;
        init_obstacle          = false;
        init_lateral_obstacles = false;

        origin_position.x = 0;
        origin_position.y = 0;
        origin_position.z = 0;  // used when passing to reset odom, encodes
                                // orientation around z axis in radians.

        m_max_base_distance = MAX_BASE_DIST;

        rot_sign_aruco_search = 1.0;
        aruco_position.x      = 0.0;
        aruco_position.y      = 0.0;

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10);      // this node will work at 10hz
        while (ros::ok()) {
            ros::spinOnce();  // each callback is called once
            update();
            r.sleep();  // we wait if the processing (ie, callback+update) has
                        // taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        // init_localization = true;
        if (init_odom && init_obstacle) {
            update_variables();

            if (current_state == searching_aruco_marker)
                process_searching_aruco_marker();
            else if (current_state == moving_to_aruco_marker)
                process_moving_to_aruco_marker();
            else
                process_avoid_lateral_crash();

            new_aruco = false;

            state_has_changed = current_state != previous_state;
            previous_state    = current_state;

        } else
            ROS_WARN("Initialize odometry");

    }  // update

    void update_variables() {
        // So far, in patrolling, there is no base (future work). Therefore
        // rotation_to_base is always 0.0
        rotation_to_base = 0.0;

        // if the robot is too close to an obstacle on its side, calculate the
        // middle between both  measures from /lateral_distances and publish a
        // new goal_to_reach
        if ((lt_obstacle_distance <= lateral_safety_threshold) ||
            (rt_obstacle_distance <= lateral_safety_threshold)) {
            ROS_WARN("Distance from the sides are below safety threshold, "
                     "changing state.");
            current_state = avoiding_lateral_crash;
        }
    }  // update_variables

    // Patrol robot - processes
    void process_searching_aruco_marker() {
        ROS_INFO("current_state: searching_aruco_marker");
        std_msgs::Float32 msg_rotation_to_do;

        // Initialize rotation to do based on odometry
        if (state_has_changed) {
            frequency               = 0;
            msg_rotation_to_do.data = rotation_to_base;
            pub_rotation_to_do.publish(msg_rotation_to_do);
            no_aruco = true;

            if (rotation_to_base > 0.0) {
                rot_sign_aruco_search = 1.0;
            } else {
                rot_sign_aruco_search = -1.0;
            }
        }

        int old_frequency = frequency;
        bool cond         = !robot_moving;

        if (cond) {
            frequency++;
            if (frequency >= (frequency_expected - 15)) {
                if (no_aruco) {
                    // Didn't see any aruco since last rotation, rotate again
                    // Reminder: this rotation must have a random factor in
                    // order to look fr the aruco. Makes no sense to always
                    // check the same places
                    std_msgs::Float32 rot;
                    rot.data = (M_PI / 2.0 + (rand() % 6) / 6.0) *
                               rot_sign_aruco_search;
                    pub_rotation_to_do.publish(rot);
                    no_aruco  = true;
                    frequency = 0;
                } else {
                    // When we see an aruco, we go towards it
                    current_state = moving_to_aruco_marker;
                }
            }
        } else {
            frequency = 0;
        }

        if (new_aruco) {
            // We see the marker, use that as the orientation.
            no_aruco         = false;
            float aruco_dist = distancePoints(origin_position, aruco_position);
            float aruco_rot  = 0.0;
            if (aruco_dist > 0) {
                aruco_rot = acos(aruco_position.x / aruco_dist);
                if (aruco_position.y < 0)
                    aruco_rot *= -1;
            }
            msg_rotation_to_do.data = aruco_rot;
            ROS_WARN("Aruco visible, correcting rotation\n");
            pub_rotation_to_do.publish(msg_rotation_to_do);
        }

        if (old_frequency != frequency)
            ROS_INFO("frequency_rotation: %i\n", frequency);
    }  // process_searching_aruco_marker

    void process_moving_to_aruco_marker() {
        ROS_INFO("current_state: moving_to_aruco_marker");

        if (state_has_changed) {
            // Send the goal to reach
            geometry_msgs::Point msg_goal_to_reach;
            frequency           = 0;
            no_aruco            = true;
            msg_goal_to_reach   = aruco_position;
            msg_goal_to_reach.z = (float)current_state;
            pub_goal_to_reach.publish(msg_goal_to_reach);
        }

        if (new_aruco) {
            no_aruco = false;
            ROS_INFO("aruco_position: (%f, %f)", aruco_position.x,
                     aruco_position.y);

            // Send the goal to reach
            geometry_msgs::Point msg_goal_to_reach;
            msg_goal_to_reach   = aruco_position;
            msg_goal_to_reach.z = (float)current_state;
            pub_goal_to_reach.publish(msg_goal_to_reach);
        }

        int old_frequency = frequency;
        if (!robot_moving && aruco_position.x <= safe_distance_from_aruco) {
            frequency++;
            if (frequency >= frequency_expected - 15) {
                current_state = searching_aruco_marker;
            }
        } else
            frequency = 0;

        if (old_frequency != frequency)
            ROS_INFO("frequency_base: %i\n", frequency);
    }  // process_moving_to_aruco_marker

    void process_avoid_lateral_crash() {
        ROS_INFO("current_state: process_avoid_lateral_crash");

        geometry_msgs::Point middle_point;
        float lt_x = lt_closest_obstacle.x, rt_x = rt_closest_obstacle.x,
              lt_y = lt_closest_obstacle.y, rt_y = rt_closest_obstacle.y;

        // Simple case: where we have both measures
        if (!(lt_x == 0 && lt_y == 0) && !(rt_x == 0 && rt_y == 0)) {
            middle_point.x = (lt_x + rt_x) / 2;
            middle_point.z = (lt_y + rt_y) / 2;
        }

        // We do not have the left measures
        if ((lt_x == 0 && lt_y == 0) && !(rt_x == 0 && rt_y == 0)) {
        }

        // We do not have the right measures
        if (!(lt_x == 0 && lt_y == 0) && (rt_x == 0 && rt_y == 0)) {
        }

    }  // process_avoid_lateral_crash

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {
        init_odom           = true;
        current_position    = o->pose.pose.position;
        current_orientation = tf::getYaw(o->pose.pose.orientation);
    }  // odomCallback

    void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {
        robot_moving = state->data;
    }  // robot_movingCallback

    void aruco_positionCallback(const geometry_msgs::Point aruco_msg) {
        new_aruco        = (aruco_msg.x > 0.1) && fabs(aruco_msg.y) > 0.01;
        aruco_position.x = aruco_msg.x;
        aruco_position.y = aruco_msg.y;
    }  // aruco_callback

    void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs) {
        init_obstacle    = true;
        closest_obstacle = *obs;
    }  // closest_obstacleCallback

    void lateral_distancesCallback(
        const patrol_robot_development::ObstacleAvoidanceMsg::ConstPtr& obs) {
        init_lateral_obstacles = true;
        lt_closest_obstacle    = obs->lt_obstacle_point;
        rt_closest_obstacle    = obs->rt_obstacle_point;
        lt_obstacle_distance   = obs->lt_obstacle_distance;
        rt_obstacle_distance   = obs->rt_obstacle_distance;
    }  // lateral_distancesCallback

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }
};

int main(int argc, char** argv) {
    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision");

    decision_node bsObject;

    ros::spin();

    return 0;
}
