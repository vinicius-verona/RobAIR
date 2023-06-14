// Signal handling
#include <signal.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>

#include "geometry_msgs/Point.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "patrol_robot_development/msg/ObstacleAvoidanceMsg.h"
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

float robair_size = 0.25;  // 0.2 for small robair

using namespace std;

class obstacle_avoidance {
private:
    ros::NodeHandle n;

    // communication with laser_scanner
    ros::Subscriber sub_scan;
    ros::Subscriber sub_scan2;

    // communication with action
    ros::Publisher pub_closest_obstacles;
    // ros::Publisher pub_closest_obstacle_marker;

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

public:
    obstacle_avoidance() {
        frame_origin.x = 0;
        frame_origin.y = 0;

        // Communication with laser scanner
        sub_scan =
            n.subscribe("scan", 1, &obstacle_avoidance::scanCallback, this);
        sub_scan2 =
            n.subscribe("scan2", 1, &obstacle_avoidance::scanCallback2, this);

        // sub_obstacle_detection =
        //     n.subscribe("closest_obstacle", 1,
        //                 &obstacle_avoidance::closest_obstacleCallback, this);

        // Communication with translation_action
        pub_closest_obstacles =
            n.advertise<patrol_robot_development::ObstacleAvoidanceMsg>(
                "lateral_distances", 1);
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
        if (init_laser && init_laser2) {
            lt_closest_obstacle.x     = range_max;
            lt_closest_obstacle.y     = range_max;
            rt_closest_obstacle.x     = range_max;
            rt_closest_obstacle.y     = range_max;
            bool lt_obstacle_detected = false;
            bool rt_obstacle_detected = false;

            float beam_angle = angle_min;
            for (int loop2 = 0; loop2 < 2; loop2++)
                for (int loop = 0; loop < nb_beams;
                     loop++, beam_angle += angle_inc) {
                    // Check the closest obstacle in the left
                    if ((current_scan[loop][loop2].y > robair_size) &&
                        (fabs(lt_closest_obstacle.y) >
                         fabs(current_scan[loop][loop2].y))) {
                        lt_closest_obstacle  = current_scan[loop][loop2];
                        lt_obstacle_detected = true;
                    }

                    // Check the closest obstacle in the right
                    if ((current_scan[loop][loop2].y < -robair_size) &&
                        (fabs(closest_obstacle.y) >
                         fabs(current_scan[loop][loop2].y))) {
                        rt_closest_obstacle  = current_scan[loop][loop2];
                        rt_obstacle_detected = true;
                    }
                }

            if (lt_obstacle_detected || rt_obstacle_detected) {
                obstacle_avoidance_msg.lt_obstacle_point = lt_closest_obstacle;
                obstacle_avoidance_msg.rt_obstacle_point = rt_closest_obstacle;
                obstacle_avoidance_msg.lt_obstacle_distance =
                    distancePoints(frame_origin, lt_closest_obstacle);
                obstacle_avoidance_msg.rt_obstacle_distance =
                    distancePoints(frame_origin, rt_closest_obstacle);
                pub_closest_obstacles.publish(obstacle_avoidance_msg);
            }
            // if (distancePoints(closest_obstacle, previous_closest_obstacle) >
            //     0.05) {
            //     ROS_INFO("closest obstacle: (%f; %f)", closest_obstacle.x,
            //              closest_obstacle.y);

            //     previous_closest_obstacle.x = closest_obstacle.x;
            //     previous_closest_obstacle.y = closest_obstacle.y;
            // }
        }
    }

    // Distance between two points
    float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
        return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
    }

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

        // Store the range and the coordinates in cartesian framework of each
        // hit
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

        // Store the range and the coordinates in cartesian framework of each
        // hit
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

    void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr &obs) {
        // init_obstacle    = true;
        // closest_obstacle = *obs;
    }  // closest_obstacleCallback
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle n;

    ROS_INFO("(obstacle_detection) PARAMETERS");

    // ros::param::get("/obstacle_avoidance_node/robot_size", robair_size);
    // ROS_INFO("(obstacle_avoidance) robot_size: %f", robair_size);

    obstacle_avoidance bsObject;

    ros::spin();

    return 0;
}
