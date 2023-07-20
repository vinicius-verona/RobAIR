// When running the decision node, it is necessary to give the following parameter:
// rosrun patrol_robot_development decision_node _map_name:="name of the map"

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include <cmath>
#include <fstream>
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

// Patrol robot processes
#define searching_aruco_marker 0
#define moving_to_aruco_marker 1
#define saving_aruco_position 2

#define safe_distance_from_aruco 0.7
#define lateral_safety_threshold 0.75
#define obstacle_safety_threshold 1

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

class generate_endpoints : ros::NodeHandle {
private:
    ros::NodeHandle n;

    // DEBUG
    ros::Publisher pub_goal_marker;
    ros::Publisher pub_localization_marker;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // Communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;
    geometry_msgs::Point closest_obstacle;
    bool init_obstacle;

    // Communication with localization
    ros::Subscriber sub_localization;
    bool new_localization;
    bool init_localization;
    geometry_msgs::Point localization;

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
    generate_endpoints() {
        // Communication with action_node
        pub_goal_to_reach =
            n.advertise<geometry_msgs::Point>("goal_to_reach",
                                              1);  // Preparing a topic to publish the position of the person
        pub_rotation_to_do =
            n.advertise<std_msgs::Float32>("rotation_to_do",
                                           1);  // Preparing a topic to publish the rotation of the person

        // Communication with odometry
        sub_odometry = n.subscribe("odom", 1, &generate_endpoints::odomCallback, this);

        // Communication with robot_moving_node
        sub_robot_moving = n.subscribe("robot_moving", 1, &generate_endpoints::robot_movingCallback, this);

        // Communication with localization_node
        sub_localization = n.subscribe("localization", 1, &generate_endpoints::localizationCallback, this);

        // Communication with aruco_node
        sub_aruco_position = n.subscribe("robair_goal", 1, &generate_endpoints::aruco_positionCallback, this);

        // Communication with obstacle_detection
        sub_obstacle_detection =
            n.subscribe("closest_obstacle", 1, &generate_endpoints::closest_obstacleCallback, this);
        pub_change_odom = n.advertise<geometry_msgs::Point>("change_odometry", 1);

        // DEBUG
        pub_goal_marker         = n.advertise<visualization_msgs::Marker>("goal_to_reach_marker", 1);
        pub_localization_marker = n.advertise<visualization_msgs::Marker>("localization_marker", 1);

        current_state  = searching_aruco_marker;
        previous_state = -1;

        new_aruco         = false;
        state_has_changed = false;
        init_odom         = false;
        init_obstacle     = false;

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
            r.sleep();        // we wait if the processing (ie, callback+update) has
                              // taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        // init_localization = true;
        if (init_odom && init_obstacle && init_localization) {
            if (current_state == searching_aruco_marker)
                process_searching_aruco_marker();
            else if (current_state == moving_to_aruco_marker)
                process_moving_to_aruco_marker();
            else if (current_state == saving_aruco_position)
                process_saving_aruco_position();

            new_aruco         = false;
            state_has_changed = current_state != previous_state;
            previous_state    = current_state;

        } else {
            ROS_WARN("Initialize odometry, obstacle detection"
                     "before starting decision node.");
            // process_navigate_in_map();
        }
    }  // update

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
            // Save the aruco position in a file with the map_name
            geometry_msgs::Point msg_goal_to_reach;
            msg_goal_to_reach.x = 0;
            msg_goal_to_reach.y = 0;
            pub_goal_to_reach.publish(msg_goal_to_reach);
            current_state = saving_aruco_position;
        }

        if (old_frequency != frequency)
            ROS_INFO("frequency_base: %i\n", frequency);
    }  // process_moving_to_aruco_marker

    void process_saving_aruco_position() {
        // Given the map name in _map_name, save the aruco position in a file with the name of the map +
        // "_aruco_positions.txt"
        ROS_INFO("current_state: saving_aruco_position");

        // Auxiliary variable
        string param_name;
        string file_name = "";

        // Search and get the parameters
        if (searchParam("map_name", param_name)) {
            getParam(param_name, file_name);

            // Open the file
            ofstream FILE;
            FILE.open(file_name + "_aruco_positions.txt", ios::app);
            FILE << localization.x << " " << localization.y << " " << current_orientation << endl;
            FILE.close();

        } else {
            ROS_ERROR("No map name given, cannot save aruco position. [generate_map_end_points node]");
            return;
        }

        current_state = searching_aruco_marker;
    }  // process_saving_aruco_position

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
    }

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
    ROS_INFO("(generate_map_end_points)");
    ros::init(argc, argv, "generate_endpoints");

    generate_endpoints bsObject;

    ros::spin();

    return 0;
}
