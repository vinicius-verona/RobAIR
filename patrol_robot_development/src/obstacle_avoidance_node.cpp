// Signal handling
#include <patrol_robot_development/ObstacleAvoidanceMsg.h>
#include <signal.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>

#include "geometry_msgs/Point.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

float robair_size = 0.6;  // 0.35 for small robair, 0.75 for philip's robair

#define x_axis_limit 1.5  // meters
#define left_angle_start (90 * M_PI / 180)
#define right_angle_start (-90 * M_PI / 180)
#define object_out_of_range 3.0

// Parameters for the Artifcial Potential Field bypass algorithm
// clang-format off
#define k_a 1              // attractive force constant
#define k_r 1              // repulsive force constant
#define step 1             // step size
#define target_radius 0.5  // radius of the target
#define p_0 0.5            // constant for the repulsive force, 
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
geometry_msgs::Point transformPoint(geometry_msgs::Point &base,
                                    geometry_msgs::Point &current_pos,
                                    float orientation) {
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

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    ros::Publisher pub_rotation_to_do;

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

    patrol_robot_development::ObstacleAvoidanceMsg obstacle_avoidance;

    geometry_msgs::Point front_closest_object;
    geometry_msgs::Point lt_obstacle_point;
    geometry_msgs::Point rt_obstacle_point;
    geometry_msgs::Point target;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

public:
    obstacle_avoidance() {
        frame_origin.x = 0;
        frame_origin.y = 0;

        // Communication with laser scanner
        sub_scan =
            n.subscribe("scan", 1, &obstacle_avoidance::scanCallback, this);
        sub_scan2 =
            n.subscribe("scan2", 1, &obstacle_avoidance::scanCallback2, this);

        // Communication with decision node
        sub_bypass =
            n.subscribe("bypass", 1, &obstacle_avoidance::bypassCallback, this);

        // Communication with action_node
        pub_goal_to_reach =
            n.advertise<geometry_msgs::Point>("goal_to_reach", 1);
        pub_rotation_to_do =
            n.advertise<std_msgs::Float32>("rotation_to_do", 1);

        init_laser  = false;
        init_laser2 = false;

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
            r.sleep();  // we wait if the processing (ie, callback+update) has
                        // taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update() {
        geometry_msgs::Point c_location;
        c_location.x = 0;
        c_location.y = 0;

        // Artificial Potential Field algorithm
        // Attractive force
        if (distancePoints(c_location, target) <= target_radius) {
            // We are close enough to the target, we stop the robot and the
            // algorithm APF
        } else {
            // We are not close enough to the target, we compute the attractive
            // force
            float attractive_force_x = target_radius * k_a *
                                       (c_location.x - target.x) /
                                       distancePoints(c_location, target);
            float attractive_force_y = target_radius * k_a *
                                       (c_location.y - target.y) /
                                       distancePoints(c_location, target);
        }

        // For each laser beam, we compute the repulsive force
        for (int loop = 0; loop < nb_beams; loop++) {
            int point_cluster_lidar1 = cluster[loop][0];
            int point_cluster_lidar2 = cluster[loop][1];
            int cluster_size_lidar1  = cluster_size[point_cluster_lidar1][0];
            int cluster_size_lidar2  = cluster_size[point_cluster_lidar2][0];

            // Repulsive force

            // // Middle point of the cluster in each lidiar
            // geometry_msgs::Point middle_point_object_lidar1 =
            //     cluster_middle[point_cluster_lidar1][0];
            // geometry_msgs::Point middle_point_object_lidar2 =
            //     cluster_middle[point_cluster_lidar2][1];
        }
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    /*
    void perform_clustering(int laser) {
        // store in the table cluster, the cluster of each hit of the laser
        // if the distance between the previous hit of the laser and the
        // current
        // one is higher than a threshold else we start a new cluster

        ROS_INFO("performing clustering for laser %i", laser);

        nb_cluster[laser] = 0;

        cluster_start[0][laser] =
            0;  // the first hit is the start of the first cluster
        cluster[0][laser]      = 0;  // the first hit belongs to the first
        cluster int nb_dynamic = 0;  // to count the number of hits of the
                                     // current cluster that are dynamic

        for (int loop = 1; loop < nb_beams; loop++)
            if (distancePoints(current_scan[loop - 1][laser],
                               current_scan[loop][laser]) < cluster_threshold) {
                cluster[loop][laser] = nb_cluster[laser];
                if (dynamic[loop][laser])
                    nb_dynamic++;
            } else {
                int current_cluster =
                    nb_cluster[laser];  // easier to read
                                        // cluster_end[current_cluster][laser]
                = loop - 1;

                int current_start = cluster_start[current_cluster][laser];
                int current_end   = cluster_end[current_cluster][laser];
                cluster_dynamic[current_cluster][laser] =
                    nb_dynamic * 100 / (current_end - current_start + 1);
                cluster_size[current_cluster][laser] =
                    distancePoints(current_scan[current_start][laser],
                                   current_scan[current_end][laser]);
                cluster_middle[current_cluster][laser].x =
                    (current_scan[current_start][laser].x +
                     current_scan[current_end][laser].x) /
                    2;
                cluster_middle[current_cluster][laser].y =
                    (current_scan[current_start][laser].y +
                     current_scan[current_end][laser].y) /
                    2;
                cluster_middle[current_cluster][laser].z =
                    (current_scan[current_start][laser].z +
                     current_scan[current_end][laser].z) /
                    2;

                nb_dynamic = 0;
                nb_cluster[laser]++;
                current_cluster++;

                cluster_start[current_cluster][laser] = loop;
                cluster[loop][laser]                  = current_cluster;
                if (dynamic[loop])
                    nb_dynamic++;
            }

        int current_cluster = nb_cluster[laser];  // easier to read
        int current_start   = cluster_start[current_cluster][laser];
        cluster_end[current_cluster][laser] = nb_beams - 1;
        int current_end = cluster_end[current_cluster][laser];
        cluster_dynamic[current_cluster][laser] =
            nb_dynamic * 100 / (current_end - current_start + 1);
        cluster_size[current_cluster][laser] =
            distancePoints(current_scan[current_start][laser],
                           current_scan[current_end][laser]);
        cluster_middle[current_cluster][laser].x =
            (current_scan[current_start][laser].x +
             current_scan[current_end][laser].x) /
            2;
        cluster_middle[current_cluster][laser].y =
            (current_scan[current_start][laser].y +
             current_scan[current_end][laser].y) /
            2;
        cluster_middle[current_cluster][laser].z =
            (current_scan[current_start][laser].z +
             current_scan[current_end][laser].z) /
            2;

        nb_cluster[laser]++;

    }  // perfor_clusterings
    // */

    // CALLBACK
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
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
            if ((scan->ranges[loop] < range_max) &&
                (scan->ranges[loop] > range_min))
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
            if ((scan->ranges[loop] < range_max) &&
                (scan->ranges[loop] > range_min))
                range[loop][1] = scan->ranges[loop];
            else
                range[loop][1] = range_max /*+ 0.2*/;

            // transform the scan in cartesian framewrok
            current_scan[loop][1].x =
                transform_laser.x + range[loop][1] * cos(beam_angle);
            current_scan[loop][1].y = range[loop][1] * sin(beam_angle);
            current_scan[loop][1].z = transform_laser.z;
        }
    }  // scanCallback2

    void bypassCallback(
        const patrol_robot_development::ObstacleAvoidanceMsg::ConstPtr &msg) {
        front_closest_object = msg->front_obstacle;
        lt_obstacle_point    = msg->lt_obstacle_point;
        rt_obstacle_point    = msg->rt_obstacle_point;
        target               = msg->goal_to_reach;
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
