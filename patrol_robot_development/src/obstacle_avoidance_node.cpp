// Signal handling
#include <limits.h>
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
    ros::Subcriber sub_odometry;

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

    patrol_robot_development::ObstacleAvoidanceMsg obstacle_avoidance;
    patrol_robot_development::ObstacleAvoidedMsg obstacle_avoided;

    geometry_msgs::Point front_closest_object;
    geometry_msgs::Point lt_obstacle_point;
    geometry_msgs::Point rt_obstacle_point;
    geometry_msgs::Point target;

    bool init_odom;
    float current_position;
    float current_orientation;
    float rotation_to_do;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

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
        pub_bypass_done    = n.advertise<std_msgs::Float32>("rotation_to_do", 1);

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
        c_location.x = 0;
        c_location.y = 0;

        // Artificial Potential Field algorithm -> multiple obstacles one target
        // Attractive force
        if (distancePoints(c_location, target) <= target_radius) {
            // We are close enough to the target, we stop the robot and the
            // algorithm APF
            obstacle_avoided.apf_in_execution = false;
            obstacle_avoided.goal_to_reach    = target;
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
                float min_dist_to_object = INT_MAX;

                // for each of the points in that cluster, we compute the distance
                // to the robot, and we keep the closest one
                for (int i = 0; i < cluster_size_lidar1; i++) {
                    float dist_to_object = distancePoint(c_location, cluster[point_cluster_lidar1][i]);
                    if (dist_to_object < min_dist_to_object) {
                        min_dist_to_object   = dist_to_object;
                        closest_point_object = cluster[point_cluster_lidar1][i];
                    }
                }

                // Repulsive force
                float repulsive_force_x = 0;
                float repulsive_force_y = 0;
                if (min_dist_to_object <= p_0) {
                    repulsive_force_x = -k_r * (1 / min_dist_to_object - 1 / p_0) *
                                        ((-closest_point_object.x / distancePoint(c_location, closest_point_object)) /
                                         pow(min_dist_to_object, 2));
                    repulsive_force_y = -k_r * (1 / min_dist_to_object - 1 / p_0) *
                                        ((-closest_point_object.y / distancePoint(c_location, closest_point_object)) /
                                         pow(min_dist_to_object, 2));
                }

                // Compute the total force
                total_force_x += repulsive_force_x;
                total_force_y += repulsive_force_y;
            }

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
                next.goal.x = 0;
                next_goal.y = 0;
            }

            obstacle_avoided.apf_in_execution = true;
            obstacle_avoided.goal_to_reach    = next_goal;
        }

        // Publish messages
        pub_bypass_done.publish(obstacle_avoided);

        if (!rotation_to_do)
            pub_goal_to_reach.publish(next_goal);
        else
            pub_rotation_to_do.publish(rotation_to_do);
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

    // CALLBACK
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

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
