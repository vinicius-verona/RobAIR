#include <geometry_msgs/Twist.h>
#include <patrol_robot_development/LateralDistancesMsg.h>
#include <patrol_robot_development/ObstacleAvoidanceMsg.h>
#include <patrol_robot_development/ObstacleAvoidedMsg.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <iostream>
#include <list>
#include <map>
#include <string>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

using namespace std;

// ROS_BREAK();

// Display goal_to_reach marker from avoid_lateral_crash
// #define DISPLAY_DEBUG 1

// Patrol robot processes
#define moving_to_aruco_marker 5
#define avoiding_lateral_crash 4
#define searching_aruco_marker 3
#define bypass_obstacle 2
#define moving_in_path 1
#define creating_path 0

// Future process
// #define avoiding_obstacle 3
// #define rotate_to_base 4
// #define return_to_base 5

#define safe_distance_from_aruco 1
#define lateral_safety_threshold 0.75
#define obstacle_safety_threshold 1

#define DEBUG_GETCHAR_ENABLED 0
#define DISPLAY_DEBUG 1
#define STATIC_TESTING_BFS 1

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
    ros::Publisher pub_localization_marker;
    ros::Publisher pub_path_marker;

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

    // Communication with localization
    ros::Subscriber sub_localization;
    bool new_localization;
    bool init_localization;
    geometry_msgs::Point localization;

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

    // Used to store the taget localization in the map frame
    geometry_msgs::Point target_map_frame;
    bool target_set;

    // force odom reset via this topic
    ros::Publisher pub_change_odom;

    float m_max_base_distance, rot_sign_aruco_search;
    float base_orientation = 0;

    int number_of_end_points;
    geometry_msgs::Point end_points_positions[1000];  // Act as vertices in the graph
    int robot_surrounding_points_id[2];
    int target_surrounding_points_id[2];
    list<geometry_msgs::Point> path_points;
    list<geometry_msgs::Point> path_left;
    int connection[1000][1000];          // Act as edges in the graph
    int vertex_connection_degree[1000];  // If a vertex has connection with 3 other vertices then its degree is 3 and we
                                         // use connection [0],[1],[2]
    bool visited_points[1000];           // Used to distinguish the points that have been visited in the graph
    // geometry_msgs::Point robot_surrounding_points[2];
    // geometry_msgs::Point target_surrounding_points[2];

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

        // Communication with localization_node
        sub_localization = n.subscribe("localization", 1, &decision_node::localizationCallback, this);

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
        pub_goal_marker         = n.advertise<visualization_msgs::Marker>("goal_to_reach_marker", 1);
        pub_localization_marker = n.advertise<visualization_msgs::Marker>("localization_marker", 1);
        pub_path_marker         = n.advertise<visualization_msgs::Marker>("path_marker", 1);

        current_state  = creating_path;
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

        // path_points = new list<geometry_msgs::Point>();

        m_max_base_distance = MAX_BASE_DIST;

        rot_sign_aruco_search = 1.0;
        aruco_position.x      = 0.0;
        aruco_position.y      = 0.0;

#ifdef STATIC_TESTING_BFS
        // Create a fake graph to test the path finding algorithm // Use the map of the plateau
        end_points_positions[0].x = -0.282858;
        end_points_positions[0].y = -1.18731;
        end_points_positions[1].x = 2.64434;
        end_points_positions[1].y = -1.18731;
        end_points_positions[2].x = 5.17791;
        end_points_positions[2].y = -7.57191;

        connection[0][0] = 1;
        connection[1][0] = 0;
        connection[1][1] = 2;
        connection[2][0] = 1;

        vertex_connection_degree[0] = 1;
        vertex_connection_degree[1] = 2;
        vertex_connection_degree[2] = 1;

        number_of_end_points = 3;
        target.x             = 3.91112;
        target.y             = -4.37961;
#endif

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10);      // this node will work at 10hz
        while (ros::ok()) {
            ros::spinOnce();  // each callback is called once
            update();
            r.sleep();        // we wait if the processing (ie, callback+update) has
                              // taken less than 0.1s (ie, 10 hz)
        }
    }

    // ~decision_node() {
    //     delete path_points;
    // }

    // UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        // init_localization = true;
        if (init_odom && init_obstacle && init_lateral_obstacles && init_localization) {
            update_variables();

            // Based on the current localization, face aruco marker if close enough or go to the closest aruco marker
            // the aruco marker positions should be kept as an array here in decision node

            // if (current_state == searching_aruco_marker && apf_in_execution == false)
            //     process_searching_aruco_marker();
            // else if (current_state == moving_to_aruco_marker && apf_in_execution == false)
            //     process_moving_to_aruco_marker();
            // else if (current_state == bypass_obstacle)
            //     process_bypass_obstacle();
            // else
            //     process_navigate_in_map();

            if (current_state == moving_in_path && apf_in_execution == false)
                process_moving_in_path();
            else if (current_state == bypass_obstacle)
                process_bypass_obstacle();
            else
                process_navigate_in_map();

            new_aruco         = false;
            state_has_changed = current_state != previous_state;
            previous_state    = current_state;

        } else {
            ROS_WARN("Initialize odometry, obstacle detection, lateral "
                     "distances, localization and obstacle avoidance"
                     "before starting decision node.");
            // process_navigate_in_map();
        }
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

        // Show localization in map frame in rviz using marker
        // display_location();

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

    void display_location() {
        // Display in RVIZ the current location of the robot
        // current_location in rviz is pink
        nb_pts            = 0;
        colors[nb_pts].r  = 1;
        colors[nb_pts].g  = 0;
        colors[nb_pts].b  = 0.4;
        colors[nb_pts].a  = 1.0;
        display[nb_pts].x = localization.x;
        display[nb_pts].y = localization.y;
        display[nb_pts].z = 0;
        nb_pts++;

        populateMarkerTopic("map", pub_localization_marker);
    }

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
                    current_state           = moving_to_aruco_marker;
                    msg_rotation_to_do.data = 0;
                    pub_rotation_to_do.publish(msg_rotation_to_do);
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
        // #ifdef DISPLAY_DEBUG
        //         nb_pts           = 0;
        //         colors[nb_pts].r = 1;
        //         colors[nb_pts].g = 1;
        //         colors[nb_pts].b = 1;
        //         colors[nb_pts].a = 1.0;
        //         display[nb_pts]  = middle_point;
        //         nb_pts++;
        //         populateMarkerTopic();
        // #endif
        current_state = 0;  // searching_aruco_marker;

    }                       // process_avoid_lateral_crash

    void process_bypass_obstacle() {
        ROS_INFO("current_state: process_bypass_obstacle");

        // if there is no aruco position, skip it
        if (aruco_position.x <= 0.00001 && aruco_position.y <= 0.00001 && target.x <= 0.00001 && target.y <= 0.00001) {
            ROS_ERROR("Aruco marker position is (0,0), there is no reason to bypass obstacle. [Decision node]");
            current_state = 0;  // searching_aruco_marker;
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

        pub_obstacle_avoidance.publish(bypass_msg);
    }

    void process_moving_in_path() {
        ROS_WARN("current_state: process_moving_in_path");

        // Publish one point of the path to follow, until it gets there checking localization,
        // keep publishing that point, after it reached a threashold distance, remove that point
        // from the path and publish the next one

        if (path_points.size() == 0) {
            ROS_WARN("Path is empty, cannot move in path.");
            return;
        }

        if (apf_in_execution) {
            ROS_ERROR("APF in execution, cannot move in path.");
            return;
        }

        geometry_msgs::Point goal = path_points.front();

        if (state_has_changed && previous_state != bypass_obstacle) {
            // Send the goal to reach
            geometry_msgs::Point msg_goal_to_reach;
            msg_goal_to_reach = path_points.front();
            path_points.pop_front();
            msg_goal_to_reach.z = (float)current_state;

            if (msg_goal_to_reach.x == 0.0 && msg_goal_to_reach.y == 0.0) {
                ROS_ERROR("Position is (0,0), cannot move to it. [Decision node - process_moving_in_path]");
                return;
            }
            pub_goal_to_reach.publish(msg_goal_to_reach);

        } else {
            // Check if the robot has reached the goal
            float distance_to_goal = distancePoints(localization, goal);
            if (distance_to_goal <= 0.5) {
                ROS_INFO("Reached goal, removing it from the path.");
                geometry_msgs::Point msg_goal_to_reach;
                msg_goal_to_reach = path_points.front();
                path_points.pop_front();

                if (msg_goal_to_reach.x == 0.0 && msg_goal_to_reach.y == 0.0) {
                    ROS_ERROR("Position is (0,0), cannot move to it. [Decision node - process_moving_in_path]");
                    return;
                }
                pub_goal_to_reach.publish(msg_goal_to_reach);
            }
        }
    }

    void process_navigate_in_map() {
        ROS_INFO("current_state: process_navigate_in_map");

#ifdef STATIC_TESTING_BFS
        localization.x = 0.282858;
        localization.y = -1.18731;
#endif

        int robot_surrounding_points[2];
        int target_surrounding_points[2];
        get_surrounding_points(localization, robot_surrounding_points);
        get_surrounding_points(target, target_surrounding_points);

        list<geometry_msgs::Point> path_pointsA;
        list<geometry_msgs::Point> path_pointsB;
        getPath(target_surrounding_points[0], robot_surrounding_points, &path_pointsA);
        getPath(target_surrounding_points[1], robot_surrounding_points, &path_pointsB);

        // copy surrounding points
        robot_surrounding_points_id[0]  = robot_surrounding_points[0];
        robot_surrounding_points_id[1]  = robot_surrounding_points[1];
        target_surrounding_points_id[0] = target_surrounding_points[0];
        target_surrounding_points_id[1] = target_surrounding_points[1];

        if (path_pointsA.size() < path_pointsB.size()) {
            path_points = path_pointsA;
        } else {
            path_points = path_pointsB;
        }

        // Copy path_points to path_left
        list<geometry_msgs::Point>::iterator it;
        for (it = path_points.begin(); it != path_points.end(); ++it) {
            path_left.push_back(*it);
        }

        // Add the goal to the path
        path_points.push_back(target);
        path_left.push_back(target);

        current_state = moving_in_path;

// Display in RVIZ the path to follow in purple and the last point in path in orange
#ifdef DISPLAY_DEBUG
        // path in rviz is purple
        nb_pts           = 0;
        colors[nb_pts].r = 0.5;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 0.5;
        colors[nb_pts].a = 1.0;
        for (it = path_points.begin(); it != path_points.end(); ++it) {
            display[nb_pts] = *it;
            nb_pts++;
        }
        populateMarkerTopic("map", pub_path_marker);

        // last point in path in rviz is orange
        nb_pts           = 0;
        colors[nb_pts].r = 1;
        colors[nb_pts].g = 0.5;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        display[nb_pts]  = target;
        nb_pts++;
        populateMarkerTopic("map", pub_path_marker);
#endif
    }

    // CALLBACKS
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void get_surrounding_points(geometry_msgs::Point point, int* surrounding_points) {
        // for each point in end_points_positions, calculate the distance to the point passed as parameter and
        // store the two closest points in an array and return it
        // geometry_msgs::Point surrounding_points[2];
        float distance_to_point1 = INT_MAX;
        float distance_to_point2 = INT_MAX;

        ROS_INFO("get_surrounding_points - point: (%f, %f)", point.x, point.y);

        for (int i = 0; i < number_of_end_points; i++) {
            // float distance_to_point = distancePoints(point, end_points_positions[i]);
            float end_point_x = end_points_positions[i].x;
            float end_point_y = end_points_positions[i].y;

            if (point.x <= end_point_x) {
                if (point.y <= end_point_y) {
                    float distance = distancePoints(point, end_points_positions[i]);
                    if (distance < distance_to_point1) {
                        distance_to_point1    = distance;
                        surrounding_points[0] = i;  // end_points_positions[i];
                    }
                } else {
                    float distance = distancePoints(point, end_points_positions[i]);
                    if (distance < distance_to_point2) {
                        distance_to_point2    = distance;
                        surrounding_points[1] = i;  // end_points_positions[i];
                    }
                }
            } else {
                if (point.y <= end_point_y) {
                    float distance = distancePoints(point, end_points_positions[i]);
                    if (distance < distance_to_point1) {
                        distance_to_point1    = distance;
                        surrounding_points[0] = i;  // end_points_positions[i];
                    }
                } else {
                    float distance = distancePoints(point, end_points_positions[i]);
                    if (distance < distance_to_point2) {
                        distance_to_point2    = distance;
                        surrounding_points[1] = i;  // end_points_positions[i];
                    }
                }
            }
        }
    }

    void getPath(int initial_vertex, int* robot_surrounding_points_id, list<geometry_msgs::Point>* path) {
        ROS_INFO("getPath - initial_vertex: %d", initial_vertex);

        // Implement BFS algorithm to find the shortest path between two points in the graph
        bool found = false;
        int found_id;

        // Visiting vertex list
        list<int> visiting_vertices;  //= new list<int>();

        // Path to be returned
        map<int, int> parents;  // = new map<int, int>();
        // list<geometry_msgs::Point> path;  // = new list<geometry_msgs::Point>();

        // Mark initial vertex as visited
        geometry_msgs::Point vertex    = end_points_positions[initial_vertex];
        visited_points[initial_vertex] = true;
        visiting_vertices.push_back(initial_vertex);

        for (int i = 0; i < number_of_end_points; i++) {
            visited_points[i] = false;
        }

        // Traverse graph
        while (!visiting_vertices.empty()) {
            int vertex_id = visiting_vertices.front();
            vertex        = end_points_positions[vertex_id];
            visiting_vertices.pop_front();
            // path.push_back(vertex);

            // Check each vertex neighbors
            for (int e = 0; e < vertex_connection_degree[vertex_id]; e++) {
                int neighbor_vertex_id               = connection[vertex_id][e];
                geometry_msgs::Point neighbor_vertex = end_points_positions[neighbor_vertex_id];

                // In case neighbor vertex is not visited, visit and insert it to the queue
                // In case it has been visited, check if the edge has already been visited.
                if (!visited_points[neighbor_vertex_id]) {
                    visiting_vertices.push_back(neighbor_vertex_id);
                    visited_points[neighbor_vertex_id] = true;
                    parents.insert(pair<int, int>(neighbor_vertex_id, vertex_id));
                }

                if (neighbor_vertex_id == robot_surrounding_points_id[0] ||
                    neighbor_vertex_id == robot_surrounding_points_id[1]) {
                    // We have found the target, stop the search
                    found    = true;
                    found_id = neighbor_vertex_id == robot_surrounding_points_id[0] ? 0 : 1;
                    break;
                }
            }

            if (found)
                break;
        }

        // Reconstruct path
        int current_vertex_id = robot_surrounding_points_id[found_id];
        while (current_vertex_id != initial_vertex) {
            path->push_front(end_points_positions[current_vertex_id]);
            current_vertex_id = parents[current_vertex_id];
        }

        if (current_vertex_id == initial_vertex) {
            path->push_back(end_points_positions[current_vertex_id]);
        }

        // return path;
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

        // Done to lock/unlock the current process in bypass_obstacle
        if (!apf_in_execution) {
            // current_state = searching_aruco_marker;
            current_state = 0;
        } else {
            current_state = bypass_obstacle;
        }
    }  // bypass_doneCallback

    void localizationCallback(const geometry_msgs::Point::ConstPtr& l) {
        // set initial base position on startup
        if (base_position.x == 0 && base_position.y == 0) {
            base_position.x  = l->x;
            base_position.y  = l->y;
            base_orientation = clamp(l->z);
        }

        new_localization    = true;
        init_localization   = true;
        localization        = *l;
        current_orientation = clamp(l->z);
    }  // localizationCallback

    void targetLocalizationCallback(const geometry_msgs::Point::ConstPtr& t) {
        target_map_frame = t;
        target_set       = true;
    }  // targetLocalizationCallback

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DEBUG
    // ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Draw the field of view and other references
    void populateMarkerReference(string frame_name, ros::Publisher publisher) {
        visualization_msgs::Marker references;

        references.header.frame_id    = frame_name;  //"map";  //"laser";
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

        // pub_goal_marker.publish(references);
        publisher.publish(references);
    }

    void populateMarkerTopic(string frame_name, ros::Publisher publisher) {
        visualization_msgs::Marker marker;

        marker.header.frame_id = frame_name;  //"map";  //"laser";
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

        // pub_goal_marker.publish(marker);
        publisher.publish(marker);
        populateMarkerReference(frame_name, publisher);
    }
};

int main(int argc, char** argv) {
    ROS_INFO("(decision_node)");
    ros::init(argc, argv, "decision");

    decision_node bsObject;

    ros::spin();

    return 0;
}
