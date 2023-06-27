// Signal handling
#include <limits.h>
#include <patrol_robot_development/ObstacleAvoidanceMsg.h>
#include <patrol_robot_development/ObstacleAvoidedMsg.h>
#include <signal.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>

#include "geometry_msgs/Point.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

// Display goal_to_reach marker
#define DISPLAY_DEBUG 1

float robair_size = 0.6;  // 0.35 for small robair, 0.75 for philip's robair

#define x_axis_limit 1.5  // meters
#define left_angle_start (90 * M_PI / 180)
#define right_angle_start (-90 * M_PI / 180)
#define object_out_of_range 3.0

// threshold for clustering
#define cluster_threshold 0.2
#define detection_threshold 0.1  // threshold for motion detection
#define dynamic_threshold 75     // to decide if a cluster is static or dynamic

// Parameters for the Artifcial Potential Field bypass algorithm
// clang-format off
#define k_a 1              // attractive force constant
#define k_r 1              // repulsive force constant
#define step 1             // step size
#define target_radius 1    // radius of the target
#define p_0 1              // constant for the repulsive force, 
                           // where P(x) is the minimum distance
                           // between the robot and the obstacle
// clang-format on
using namespace std;

float clamp(float orientation) {
    while (orientation > M_PI)
        orientation -= 2 * M_PI;

    while (orientation < -M_PI)
        orientation += 2 * M_PI;

    return orientation;
}

// Transofrm the position of the base in the cartesian local frame of robot
// Input: base_position: position of the base in the map frame
//        current_position: position of the robot in the map frame
//        orientation: orientation of the robot in the map frame
// Output: new_base_point: position of the base in the local frame of the robot
geometry_msgs::Point transformPoint(geometry_msgs::Point &base, geometry_msgs::Point &current_pos, float orientation) {
    geometry_msgs::Point new_base_point;
    float x = base.x - current_pos.x;
    float y = base.y - current_pos.y;

    new_base_point.x = x * cos(orientation) + y * sin(orientation);
    new_base_point.y = -x * sin(orientation) + y * cos(orientation);
    return new_base_point;
}

class obstacle_avoidance {
private:
    ros::NodeHandle n;

    // communication with laser_scanner
    ros::Subscriber sub_scan;
    ros::Subscriber sub_scan2;

    // Communication with odometry
    ros::Subscriber sub_odometry;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    ros::Publisher pub_rotation_to_do;

    // Communication with Decision node
    ros::Subscriber sub_bypass;
    ros::Publisher pub_bypass_done;

    // to store, process and display both laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000][2];
    geometry_msgs::Point current_scan[1000][2];
    bool init_laser, init_laser2;
    geometry_msgs::Point transform_laser;

    geometry_msgs::Point previous_closest_obstacle;
    geometry_msgs::Point lt_closest_obstacle, rt_closest_obstacle;
    geometry_msgs::Point frame_origin;

    patrol_robot_development::ObstacleAvoidanceMsg obstacle_avoidance_msg;
    patrol_robot_development::ObstacleAvoidedMsg obstacle_avoided_msg;

    geometry_msgs::Point front_closest_object;
    geometry_msgs::Point lt_obstacle_point;
    geometry_msgs::Point rt_obstacle_point;
    geometry_msgs::Point target;

    bool init_odom;
    geometry_msgs::Point current_position;
    float current_orientation;
    float rotation_to_do;

    // to perform detection of motion
    bool stored_background;
    float background[1000][2];
    bool dynamic[1000][2];
    bool init_robot;
    bool current_robot_moving;

    // to perform clustering
    int nb_cluster[2];                                    // number of cluster
    int cluster[1000][2];                                 // to store for each hit, the cluster it belongs to
    float cluster_size[1000][2];                          // to store the size of each cluster
    geometry_msgs::Point closest_cluster_point[1000][2];  // to store for each cluster, the closest point to the robot
    geometry_msgs::Point cluster_middle[1000][2];         // to store the middle of each cluster
    float cluster_dynamic[1000][2];                       // to store the percentage of the cluster that is dynamic
    int cluster_start[1000][2], cluster_end[1000][2];
    bool local_minimum[1000][2];

    ros::Subscriber sub_robot_moving;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

    // DEBUG
    ros::Publisher pub_goal_marker;

public:
    obstacle_avoidance() {
        frame_origin.x = 0;
        frame_origin.y = 0;

        // Communication with laser scanner
        sub_scan  = n.subscribe("scan", 1, &obstacle_avoidance::scanCallback, this);
        sub_scan2 = n.subscribe("scan2", 1, &obstacle_avoidance::scanCallback2, this);

        // Communication with odometry
        sub_odometry = n.subscribe("odom", 1, &obstacle_avoidance::odometryCallback, this);

        // Communication with decision node
        sub_bypass = n.subscribe("bypass", 1, &obstacle_avoidance::bypassCallback, this);

        // Communication with action_node
        pub_goal_to_reach  = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);
        pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 1);
        pub_bypass_done    = n.advertise<patrol_robot_development::ObstacleAvoidedMsg>("bypass_done", 1);

        sub_robot_moving = n.subscribe("robot_moving", 1, &obstacle_avoidance::robot_movingCallback, this);

        // DEBUG
        pub_goal_marker = n.advertise<visualization_msgs::Marker>("goal_to_reach_marker", 1);

        init_laser  = false;
        init_laser2 = false;
        init_odom   = false;

        tf::StampedTransform transform;
        tf::TransformListener listener;

        transform_laser.x = -.35;
        transform_laser.y = 0;
        transform_laser.z = 1.2;

        // INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10);      // this node will run at 10hz
        while (ros::ok()) {
            ros::spinOnce();  // each callback is called once to collect new
                              // data: laser + robot_moving
            update();         // processing of data
            r.sleep();        // we wait if the processing (ie, callback+update) has
                              // taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        if (!init_laser || !init_laser2 || !init_odom) {
            ROS_WARN("Waiting for laser and odometry to be running...");
            return;
        }
        float total_force_x;
        float total_force_y;
        geometry_msgs::Point next_goal;
        geometry_msgs::Point c_location;
        c_location.x   = 0;
        c_location.y   = 0;
        rotation_to_do = 0;

        // Stop the robot if it is moving
        if (current_robot_moving) {
            pub_goal_to_reach.publish(c_location);
        }

        detect_motion(0);
        detect_motion(1);
        perform_clustering(0);
        perform_clustering(1);

        // Print as error the target, the c_location and the distance between these two
        ROS_ERROR("Target: (%f, %f)\n C_location: (%f, %f)\n Distance: %f", target.x, target.y, c_location.x,
                  c_location.y, distancePoints(c_location, target));

        // Artificial Potential Field algorithm -> multiple obstacles one target
        // Attractive force
        if (distancePoints(c_location, target) <= target_radius) {
            // We are close enough to the target, we stop the robot and the
            // algorithm APF
            obstacle_avoided_msg.apf_in_execution = false;
            obstacle_avoided_msg.goal_to_reach    = target;
        } else {
            // We are not close enough to the target, we compute the attractive
            // force
            float attractive_force_x =
                target_radius * k_a * (c_location.x - target.x) / distancePoints(c_location, target);
            float attractive_force_y =
                target_radius * k_a * (c_location.y - target.y) / distancePoints(c_location, target);

            total_force_x = attractive_force_x;
            total_force_y = attractive_force_y;

            // For each laser beam, we compute the repulsive force
            for (int loop = 0; loop < nb_beams; loop++) {
                int point_cluster_lidar1 = cluster[loop][0];
                int point_cluster_lidar2 = cluster[loop][1];
                int cluster_size_lidar1  = cluster_size[point_cluster_lidar1][0];
                int cluster_size_lidar2  = cluster_size[point_cluster_lidar2][0];

                // Find closest object to the robot for a given cluster
                geometry_msgs::Point closest_point_object;
                geometry_msgs::Point closest_point_object_1 = closest_cluster_point[point_cluster_lidar1][0];
                geometry_msgs::Point closest_point_object_2 = closest_cluster_point[point_cluster_lidar1][1];
                float min_dist_to_object_1                  = distancePoints(c_location, closest_point_object_1);
                float min_dist_to_object_2                  = distancePoints(c_location, closest_point_object_2);
                float min_dist_to_object                    = 0;

                if (min_dist_to_object_1 < min_dist_to_object_2) {
                    closest_point_object = closest_point_object_1;
                    min_dist_to_object   = min_dist_to_object_1;
                } else {
                    closest_point_object = closest_point_object_2;
                    min_dist_to_object   = min_dist_to_object_2;
                }

                // Repulsive force
                float repulsive_force_x = 0;
                float repulsive_force_y = 0;
                if (min_dist_to_object <= p_0 * 1.5) {
                    repulsive_force_x = -k_r * (1 / min_dist_to_object - 1 / p_0) *
                                        ((-closest_point_object.x / distancePoints(c_location, closest_point_object)) /
                                         pow(min_dist_to_object, 2));
                    repulsive_force_y = -k_r * (1 / min_dist_to_object - 1 / p_0) *
                                        ((-closest_point_object.y / distancePoints(c_location, closest_point_object)) /
                                         pow(min_dist_to_object, 2));
                }

                // Compute the total force
                total_force_x += repulsive_force_x;
                total_force_y += repulsive_force_y;
            }

            ROS_WARN("total_force: (%f, %f)\n times step = (%f, %f)", total_force_x, total_force_y,
                     total_force_x * step, total_force_y * step);

            // Compute the goal to reach
            next_goal.x = c_location.x - (step * total_force_x);
            next_goal.y = c_location.y - (step * total_force_y);

            // TODO: check if the x axis for the goal to reach is greater in absolute value than the closest obstacle
            // in front, we have to rotate the robot

            // If the new x coordinate is negative, we just rotate to face the point.
            if (next_goal.x < 0) {
                rotation_to_do = acos(next_goal.x / distancePoints(c_location, next_goal));
                if (next_goal.y < 0) {
                    rotation_to_do = -rotation_to_do;
                }
                next_goal.x = 0;
                next_goal.y = 0;
            }

            obstacle_avoided_msg.apf_in_execution = true;
            obstacle_avoided_msg.goal_to_reach    = next_goal;
        }

#ifdef DISPLAY_DEBUG

        if (next_goal.x == 0.0 && next_goal.y == 0.0) {
            ROS_ERROR("Aruco marker position is (0,0), cannot move to it. [Obstacle avoidance node]");
            return;
        }

        // Publish messages
        pub_bypass_done.publish(obstacle_avoided_msg);

        if (rotation_to_do == 0)
            pub_goal_to_reach.publish(next_goal);
        else {
            std_msgs::Float32 msg_rotation_to_do;
            msg_rotation_to_do.data = rotation_to_do;
            pub_rotation_to_do.publish(msg_rotation_to_do);
        }

#endif
#ifdef DISPLAY_DEBUG
        // populate marker with goal_to_reach and target
        // goal_to_reach is pink and target is blue-greenish
        nb_pts           = 0;
        colors[nb_pts].r = 1;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;
        display[nb_pts]  = next_goal;
        nb_pts++;

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        display[nb_pts]  = target;
        nb_pts++;
        populateMarkerTopic();
#endif
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    void detect_motion(int laser) {
        // for each hit, compare the current range with the background to detect motion

        ROS_INFO("detect_motion for %i laser", laser);
        // nb_pts = 0;
        for (int loop = 0; loop < nb_beams; loop++)
            if ((!current_robot_moving) &&
                (((background[loop][laser] - range[loop][laser]) >
                  detection_threshold) ||  // we r getting closer to the robot
                 (((range[loop][laser] - background[loop][laser]) > detection_threshold) &&
                  ((range[loop][laser] - background[loop][laser]) < 2 * detection_threshold)))) {
                dynamic[loop][laser] = true;
            } else
                dynamic[loop][laser] = false;

        ROS_INFO("%i points are dynamic", nb_pts);
        // populateMarkerTopic();
        // getchar();

    }  // detect_motion

    void perform_clustering(int laser) {
        // store in the table cluster, the cluster of each hit of the laser
        // if the distance between the previous hit of the laser and the current one is higher than a threshold
        // else we start a new cluster

        ROS_INFO("performing clustering for laser %i", laser);

        nb_cluster[laser] = 0;

        cluster_start[0][laser] = 0;  // the first hit is the start of the first cluster
        cluster[0][laser]       = 0;  // the first hit belongs to the first cluster
        int nb_dynamic          = 0;  // to count the number of hits of the current cluster that are dynamic
        float closest_point     = INT_MAX;

        for (int loop = 1; loop < nb_beams; loop++) {
            if (distancePoints(current_scan[loop - 1][laser], current_scan[loop][laser]) < cluster_threshold) {
                cluster[loop][laser] = nb_cluster[laser];
                if (dynamic[loop][laser])
                    nb_dynamic++;
            } else {
                int current_cluster                 = nb_cluster[laser];  // easier to read
                cluster_end[current_cluster][laser] = loop - 1;

                int current_start                       = cluster_start[current_cluster][laser];
                int current_end                         = cluster_end[current_cluster][laser];
                cluster_dynamic[current_cluster][laser] = nb_dynamic * 100 / (current_end - current_start + 1);
                cluster_size[current_cluster][laser] =
                    distancePoints(current_scan[current_start][laser], current_scan[current_end][laser]);
                cluster_middle[current_cluster][laser].x =
                    (current_scan[current_start][laser].x + current_scan[current_end][laser].x) / 2;
                cluster_middle[current_cluster][laser].y =
                    (current_scan[current_start][laser].y + current_scan[current_end][laser].y) / 2;
                cluster_middle[current_cluster][laser].z =
                    (current_scan[current_start][laser].z + current_scan[current_end][laser].z) / 2;

                ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %f",
                         current_cluster, cluster_middle[current_cluster][laser].x,
                         cluster_middle[current_cluster][laser].y, current_start, current_scan[current_start][laser].x,
                         current_scan[current_start][laser].y, current_end, current_scan[current_end][laser].x,
                         current_scan[current_end][laser].y, cluster_size[current_cluster][laser], nb_dynamic,
                         cluster_dynamic[current_cluster][laser]);

                nb_dynamic = 0;
                nb_cluster[laser]++;
                current_cluster++;
                closest_point = INT_MAX;

                cluster_start[current_cluster][laser] = loop;
                cluster[loop][laser]                  = current_cluster;
                if (dynamic[loop])
                    nb_dynamic++;
            }

            // If the current scan is closer than the closest point for that cluster, change the value stored
            // The variable where we store the info is the one closest_cluster_point
            geometry_msgs::Point robot_position;
            robot_position.x = 0;
            robot_position.y = 0;

            float dist_point_robot = distancePoints(current_scan[loop][laser], robot_position);

            if (dist_point_robot < closest_point) {
                closest_cluster_point[nb_cluster[laser]][laser] = current_scan[loop][laser];
            }
        }

        int current_cluster                 = nb_cluster[laser];  // easier to read
        int current_start                   = cluster_start[current_cluster][laser];
        cluster_end[current_cluster][laser] = nb_beams - 1;
        int current_end                     = cluster_end[current_cluster][laser];

        cluster_dynamic[current_cluster][laser] = nb_dynamic * 100 / (current_end - current_start + 1);
        cluster_size[current_cluster][laser] =
            distancePoints(current_scan[current_start][laser], current_scan[current_end][laser]);
        cluster_middle[current_cluster][laser].x =
            (current_scan[current_start][laser].x + current_scan[current_end][laser].x) / 2;
        cluster_middle[current_cluster][laser].y =
            (current_scan[current_start][laser].y + current_scan[current_end][laser].y) / 2;
        cluster_middle[current_cluster][laser].z =
            (current_scan[current_start][laser].z + current_scan[current_end][laser].z) / 2;

        ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %f", current_cluster,
                 cluster_middle[current_cluster][laser].x, cluster_middle[current_cluster][laser].y, current_start,
                 current_scan[current_start][laser].x, current_scan[current_start][laser].y, current_end,
                 current_scan[current_end][laser].x, current_scan[current_end][laser].y,
                 cluster_size[current_cluster][laser], nb_dynamic, cluster_dynamic[current_cluster][laser]);

        nb_cluster[laser]++;

    }  // perfor_clustering

    // CALLBACK
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

    // void odometryCallback(const nav_msgs::Odometry::ConstPtr &o) {
    //     init_odom           = true;
    //     current_position    = o->pose.pose.position;
    //     current_orientation = tf::getYaw(o->pose.pose.orientation);
    // }  // odomCallback
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &o) {
        init_odom           = true;
        current_position    = o->pose.pose.position;
        current_orientation = tf::getYaw(o->pose.pose.orientation);
    }  // odomCallback

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
        init_laser = true;

        // Store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams  = ((-1 * angle_min) + angle_max) / angle_inc;

        // Store the range and the coordinates in cartesian framework of
        // each hit
        float beam_angle = angle_min;
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
            if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
                range[loop][0] = scan->ranges[loop];
            else
                range[loop][0] = range_max;

            // Transform the scan in cartesian framewrok
            current_scan[loop][0].x = range[loop][0] * cos(beam_angle);
            current_scan[loop][0].y = range[loop][0] * sin(beam_angle);
            current_scan[loop][0].z = 0.0;
        }

    }  // scanCallback

    void scanCallback2(const sensor_msgs::LaserScan::ConstPtr &scan) {
        init_laser2 = true;

        // Store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams  = ((-1 * angle_min) + angle_max) / angle_inc;

        // Store the range and the coordinates in cartesian framework of
        // each hit
        float beam_angle = angle_min;
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
            if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
                range[loop][1] = scan->ranges[loop];
            else
                range[loop][1] = range_max /*+ 0.2*/;

            // transform the scan in cartesian framewrok
            current_scan[loop][1].x = transform_laser.x + range[loop][1] * cos(beam_angle);
            current_scan[loop][1].y = range[loop][1] * sin(beam_angle);
            current_scan[loop][1].z = transform_laser.z;
        }
    }  // scanCallback2

    void bypassCallback(const patrol_robot_development::ObstacleAvoidanceMsg::ConstPtr &msg) {
        front_closest_object = msg->front_obstacle;
        lt_obstacle_point    = msg->lt_obstacle_point;
        rt_obstacle_point    = msg->rt_obstacle_point;
        target               = msg->goal_to_reach;
    }

    void robot_movingCallback(const std_msgs::Bool::ConstPtr &state) {
        init_robot           = true;
        current_robot_moving = state->data;
    }  // robot_movingCallback
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;

    ROS_INFO("(obstacle_avoidance) PARAMETERS");

    ros::param::get("/obstacle_avoidance_node/robot_size", robair_size);
    ROS_INFO("(obstacle_avoidance) robot_size: %f", robair_size);

    obstacle_avoidance bsObject;

    ros::spin();

    return 0;
}
