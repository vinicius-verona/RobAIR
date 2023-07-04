#include <geometry_msgs/Twist.h>
#include <patrol_robot_development/LateralDistancesMsg.h>
#include <patrol_robot_development/ObstacleAvoidanceMsg.h>
#include <patrol_robot_development/ObstacleAvoidedMsg.h>
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
#include "visualization_msgs/Marker.h"

// Display goal_to_reach marker from avoid_lateral_crash
// #define DISPLAY_DEBUG 1

// Patrol robot processes
#define searching_aruco_marker 0
#define moving_to_aruco_marker 1
#define avoiding_lateral_crash 2
#define bypass_obstacle 3

// Future process
// #define avoiding_obstacle 3
// #define rotate_to_base 4
// #define return_to_base 5

#define safe_distance_from_aruco 1
#define lateral_safety_threshold 0.75
#define obstacle_safety_threshold 1

#define DEBUG_GETCHAR_ENABLED 0

// Philip Plateau Test
#define base_position_x 0
#define base_position_y 0

#define frequency_expected 25
#define MAX_BASE_DIST 5.0
#define min_angle 0.1      // 0.1 rad = 5 degrees

#define ARUCO_MODE 1       // if 0, use only odometry
#define AUDIO_COOLDOWN 30  // number of node cycles to wait before saying the same thing again

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

    // DEBUG
    ros::Publisher pub_goal_marker;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // Communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;
    geometry_msgs::Point closest_obstacle;
    bool init_obstacle;

    // Communication with lateral_distances_node
    ros::Subscriber sub_lateral_distances;
    geometry_msgs::Point lt_closest_obstacle;
    geometry_msgs::Point rt_closest_obstacle;
    bool init_lateral_obstacles;
    float lt_obstacle_distance;
    float rt_obstacle_distance;

    // Communication with obstacle_avoidance
    ros::Subscriber sub_obstacle_avoidance;
    ros::Publisher pub_obstacle_avoidance;
    patrol_robot_development::ObstacleAvoidanceMsg bypass_msg;
    geometry_msgs::Point bypass_done_target;
    geometry_msgs::Point target;
    bool apf_in_execution;

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

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

public:
    decision_node() {
        // Communication with action_node
        pub_goal_to_reach =
            n.advertise<geometry_msgs::Point>("goal_to_reach",
                                              1);  // Preparing a topic to publish the position of the person
        pub_rotation_to_do =
            n.advertise<std_msgs::Float32>("rotation_to_do",
                                           1);  // Preparing a topic to publish the rotation of the person

        // Communication with odometry
        sub_odometry = n.subscribe("odom", 1, &decision_node::odomCallback, this);

        // Communication with robot_moving_node
        sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

        // Communication with aruco_node
        sub_aruco_position = n.subscribe("robair_goal", 1, &decision_node::aruco_positionCallback, this);

        // Communication with obstacle_detection
        sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &decision_node::closest_obstacleCallback, this);

        // Communication with lateral_distances
        sub_lateral_distances = n.subscribe("lateral_distances", 1, &decision_node::lateral_distancesCallback, this);

        // Communication with obstacle_avoidance
        sub_obstacle_avoidance = n.subscribe("bypass_done", 1, &decision_node::bypass_doneCallback, this);
        pub_obstacle_avoidance = n.advertise<patrol_robot_development::ObstacleAvoidanceMsg>("bypass", 1);

        pub_change_odom = n.advertise<geometry_msgs::Point>("change_odometry", 1);

        // DEBUG
        pub_goal_marker = n.advertise<visualization_msgs::Marker>("goal_to_reach_marker", 1);

        current_state  = searching_aruco_marker;
        previous_state = -1;

        new_aruco              = false;
        state_has_changed      = false;
        init_odom              = false;
        init_obstacle          = false;
        init_lateral_obstacles = false;
        apf_in_execution       = false;

        origin_position.x = 0;
        origin_position.y = 0;
        origin_position.z = 0;  // used when passing to reset odom, encodes
                                // orientation around z axis in radians.

        target.x = 0;
        target.y = 0;

        m_max_base_distance = MAX_BASE_DIST;

        rot_sign_aruco_search = 1.0;
        aruco_position.x      = 0.0;
        aruco_position.y      = 0.0;

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10);      // this node will work at 10hz
        while (ros::ok()) {
            ros::spinOnce();  // each callback is called once
            update();
            r.sleep();        // we wait if the processing (ie, callback+update) has
                              // taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        // init_localization = true;
        if (init_odom && init_obstacle && init_lateral_obstacles) {
            update_variables();

            if (current_state == searching_aruco_marker && apf_in_execution == false)
                process_searching_aruco_marker();
            else if (current_state == moving_to_aruco_marker && apf_in_execution == false)
                process_moving_to_aruco_marker();
            // else if (current_state == avoiding_lateral_crash)
            //     process_avoid_lateral_crash();
            else
                process_bypass_obstacle();

            new_aruco         = false;
            state_has_changed = current_state != previous_state;
            previous_state    = current_state;

        } else
            ROS_WARN("Initialize odometry, obstacle detection, lateral "
                     "distances and obstacle avoidance before starting "
                     "decision node.");

    }  // update

    void update_variables() {
        // So far, in patrolling, there is no base (future work). Therefore
        // rotation_to_base is always 0.0
        rotation_to_base = 0.0;

        // if the robot is too close to an obstacle on its side, calculate the
        // middle between both  measures from /lateral_distances and publish a
        // new goal_to_reach
        // if ((lt_obstacle_distance <= lateral_safety_threshold) || (rt_obstacle_distance <= lateral_safety_threshold))
        // {
        //     ROS_WARN("Distance from the sides are below safety threshold, "
        //              "changing state.");
        //     current_state = avoiding_lateral_crash;
        // }

        // If the robot finds an obstacle in the way between target and current
        // position apply an algorithm (APF), for example, to bypass it
        if (closest_obstacle.x <= obstacle_safety_threshold && closest_obstacle.y <= obstacle_safety_threshold &&
            previous_state == moving_to_aruco_marker) {
            ROS_WARN("Obstacle close to the robot, applying bypassing algorithm.");
            current_state = bypass_obstacle;

            // Stop the robot if it is moving
            if (robot_moving) {
                geometry_msgs::Point center;
                center.x = 0;
                center.y = 0;

                pub_goal_to_reach.publish(center);
            }
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
                    rot.data = (M_PI / 2.0 + (rand() % 6) / 6.0) * rot_sign_aruco_search;
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

        if (aruco_position.x != 0 && aruco_position.y != 0) {
            target = aruco_position;
        }

        if (old_frequency != frequency)
            ROS_INFO("frequency_rotation: %i\n", frequency);
    }  // process_searching_aruco_marker

    void process_moving_to_aruco_marker() {
        ROS_INFO("current_state: moving_to_aruco_marker");

        if (apf_in_execution) {
            ROS_ERROR("APF in execution, cannot move to aruco marker.");
            return;
        }

        if (state_has_changed) {
            // Send the goal to reach
            geometry_msgs::Point msg_goal_to_reach;
            frequency           = 0;
            no_aruco            = true;
            msg_goal_to_reach   = aruco_position;
            msg_goal_to_reach.z = (float)current_state;

            if (msg_goal_to_reach.x == 0.0 && msg_goal_to_reach.y == 0.0) {
                ROS_ERROR("Aruco marker position is (0,0), cannot move to it. [Decision node]");
                return;
            }
            pub_goal_to_reach.publish(msg_goal_to_reach);
        }

        if (new_aruco) {
            no_aruco = false;
            ROS_INFO("aruco_position: (%f, %f)", aruco_position.x, aruco_position.y);

            // Send the goal to reach
            geometry_msgs::Point msg_goal_to_reach;
            msg_goal_to_reach   = aruco_position;
            msg_goal_to_reach.z = (float)current_state;

            if (msg_goal_to_reach.x == 0.0 && msg_goal_to_reach.y == 0.0) {
                ROS_ERROR("Aruco marker position is (0,0), cannot move to it. [Decision node]");
                return;
            }
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
        geometry_msgs::Point msg_goal_to_reach;
        float lt_x = lt_closest_obstacle.x, rt_x = rt_closest_obstacle.x, lt_y = lt_closest_obstacle.y,
              rt_y = rt_closest_obstacle.y;

        // Simple case: where we have both measures
        if (!(lt_x == 0 && lt_y == 0) && !(rt_x == 0 && rt_y == 0)) {
            middle_point.x = ((lt_x + rt_x) / 2) + 1;
            middle_point.y = (lt_y + rt_y) / 2;
        }

        // We do not have the left measures
        if ((lt_x == 0 && lt_y == 0) && !(rt_x == 0 && rt_y == 0)) {
            middle_point.x = rt_x + 1;
            middle_point.y = rt_y + 2;
        }

        // We do not have the right measures
        if (!(lt_x == 0 && lt_y == 0) && (rt_x == 0 && rt_y == 0)) {
            middle_point.x = lt_x + 1;
            middle_point.y = lt_y - 2;
        }

        msg_goal_to_reach = middle_point;
        pub_goal_to_reach.publish(msg_goal_to_reach);

/* Display in RVIZ the point where the robair is supposed to go */
// goal_to_reach in rviz is white
#ifdef DISPLAY_DEBUG
        nb_pts           = 0;
        colors[nb_pts].r = 1;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;
        display[nb_pts]  = middle_point;
        nb_pts++;
        populateMarkerTopic();
#endif
        current_state = searching_aruco_marker;

    }  // process_avoid_lateral_crash

    void process_bypass_obstacle() {
        ROS_INFO("current_state: process_bypass_obstacle");

        // if there is no aruco position, skip it
        if (aruco_position.x <= 0.00001 && aruco_position.y <= 0.00001 && target.x <= 0.00001 && target.y <= 0.00001) {
            ROS_ERROR("Aruco marker position is (0,0), there is no reason to bypass obstacle. [Decision node]");
            current_state = searching_aruco_marker;
            return;
        }

        if (!apf_in_execution) {
            target                   = aruco_position;
            bypass_msg.goal_to_reach = target;
        } else {
            bypass_msg.goal_to_reach = bypass_done_target;
        }

        bypass_msg.front_obstacle    = closest_obstacle;
        bypass_msg.lt_obstacle_point = lt_closest_obstacle;
        bypass_msg.rt_obstacle_point = rt_closest_obstacle;
        bypass_msg.target_found      = aruco_position.x == 0 && aruco_position.y == 0 ? false : true;
        bypass_msg.enable_apf        = true;

        // ROS_WARN("-- Message sent to bypass obstacle -- "
        //          "\n\tGoal  = (%f, %f)"
        //          "\n\tFront = (%f, %f)"
        //          "\n\tLeft  = (%f, %f)"
        //          "\n\tRight = (%f, %f)"
        //          "\n\tFound = (%d)"
        //          "\n\tAruco = (%f, %f)"
        //          "\n\tAPF   = (%d)"
        //          "\n\tTarget= (%f, %f)"
        //          "\n\tBypass= (%f, %f)",
        //          bypass_msg.goal_to_reach.x, bypass_msg.goal_to_reach.y, bypass_msg.front_obstacle.x,
        //          bypass_msg.front_obstacle.y, bypass_msg.lt_obstacle_point.x, bypass_msg.lt_obstacle_point.y,
        //          bypass_msg.rt_obstacle_point.x, bypass_msg.rt_obstacle_point.y, bypass_msg.target_found,
        //          aruco_position.x, aruco_position.y, apf_in_execution, target.x, target.y, bypass_done_target.x,
        //          bypass_done_target.y);
        // ROS_BREAK();

        pub_obstacle_avoidance.publish(bypass_msg);
    }

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

    void lateral_distancesCallback(const patrol_robot_development::LateralDistancesMsg::ConstPtr& obs) {
        init_lateral_obstacles = true;
        lt_closest_obstacle    = obs->lt_obstacle_point;
        rt_closest_obstacle    = obs->rt_obstacle_point;
        lt_obstacle_distance   = obs->lt_obstacle_distance;
        rt_obstacle_distance   = obs->rt_obstacle_distance;
    }  // lateral_distancesCallback

    void bypass_doneCallback(const patrol_robot_development::ObstacleAvoidedMsg::ConstPtr& obs) {
        bypass_done_target = obs->goal_to_reach;
        apf_in_execution   = obs->apf_in_execution;

        ROS_WARN("--- Bypass done callback - data --- "
                 "\n\tGoal  = (%f, %f)"
                 "\n\tAPF   = (%d)",
                 bypass_done_target.x, bypass_done_target.y, apf_in_execution);
        ROS_BREAK();

        // Done to lock/unlock the current process in bypass_obstacle
        if (!apf_in_execution) {
            current_state = searching_aruco_marker;
        } else {
            current_state = bypass_obstacle;
        }
    }  // bypass_doneCallback

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DEBUG
    // ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Draw the field of view and other references
    void populateMarkerReference() {
        visualization_msgs::Marker references;

        references.header.frame_id    = "laser";
        references.header.stamp       = ros::Time::now();
        references.ns                 = "example";
        references.id                 = 1;
        references.type               = visualization_msgs::Marker::LINE_STRIP;
        references.action             = visualization_msgs::Marker::ADD;
        references.pose.orientation.w = 1;

        references.scale.x = 0.02;

        references.color.r = 1.0f;
        references.color.g = 1.0f;
        references.color.b = 1.0f;
        references.color.a = 1.0;
        geometry_msgs::Point v;

        v.x = 0.02 * cos(-2.356194);
        v.y = 0.02 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        v.x = 5.6 * cos(-2.356194);
        v.y = 5.6 * sin(-2.356194);
        v.z = 0.0;
        references.points.push_back(v);

        float beam_angle = -2.356194 + 0.006136;
        // first and last beam are already included
        for (int i = 0; i < 723; i++, beam_angle += 0.006136) {
            v.x = 5.6 * cos(beam_angle);
            v.y = 5.6 * sin(beam_angle);
            v.z = 0.0;
            references.points.push_back(v);
        }

        v.x = 5.6 * cos(2.092350);
        v.y = 5.6 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        v.x = 0.02 * cos(2.092350);
        v.y = 0.02 * sin(2.092350);
        v.z = 0.0;
        references.points.push_back(v);

        pub_goal_marker.publish(references);
    }

    void populateMarkerTopic() {
        visualization_msgs::Marker marker;

        marker.header.frame_id = "laser";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "example";
        marker.id              = 0;
        marker.type            = visualization_msgs::Marker::POINTS;
        marker.action          = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;

        marker.color.a = 1.0;

        // ROS_INFO("%i points to display", nb_pts);
        for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            // ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y,
            // p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

        pub_goal_marker.publish(marker);
        populateMarkerReference();
    }
};

int main(int argc, char** argv) {
    ROS_INFO("(decision_node)");
    ros::init(argc, argv, "decision");

    decision_node bsObject;

    ros::spin();

    return 0;
}
