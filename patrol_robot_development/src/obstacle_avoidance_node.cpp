// Signal handling
#include <signal.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>

#include "geometry_msgs/Point.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "patrol_robot_development/ObstacleAvoidanceMsg.h"
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
    ros::Publisher pub_closest_obstacle;
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
    geometry_msgs::Point closest_obstacle;

    patrol_robot_development::ObstacleAvoidanceMsg obstacle_avoidance_msg;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

public:
    obstacle_avoidance() {
        // Communication with laser scanner
        sub_scan =
            n.subscribe("scan", 1, &obstacle_detection::scanCallback, this);
        sub_scan2 =
            n.subscribe("scan2", 1, &obstacle_detection::scanCallback2, this);

        // Communication with translation_action
        pub_closest_obstacle =
            n.advertise<patrol_robot_development::ObstacleAvoidanceMsg>(
                "lateral_distances", 1);
        // pub_closest_obstacle_marker =
        // n.advertise<visualization_msgs::Marker>("closest_obstacle_marker",
        // 1); // Preparing a topic to publish our results. This will be used by
        // the visualization tool rviz
        init_laser  = false;
        init_laser2 = false;

        tf::StampedTransform transform;
        tf::TransformListener listener;

        transform_laser.x = -.35;
        transform_laser.y = 0;
        transform_laser.z = 1.2;

        /*  try {
              listener.waitForTransform("/laser", "/laser2", ros::Time(0),
          ros::Duration(10.0) ); listener.lookupTransform("/laser", "/laser2",
          ros::Time(0), transform); transform_laser.x =
          transform.getOrigin().x(); transform_laser.y =
          transform.getOrigin().y(); transform_laser.z =
          transform.getOrigin().z(); } catch (tf::TransformException ex) {
              ROS_ERROR("%s",ex.what());
          }*/

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
        // ROS_INFO("init_laser: %i, init_laser2: %i", init_laser, init_laser2);
        if (init_laser && init_laser2) {
            closest_obstacle.x     = range_max;
            closest_obstacle.y     = range_max;
            bool obstacle_detected = false;
            // ROS_INFO("closest obstacle: (%f; %f)", closest_obstacle.x,
            // closest_obstacle.y);

            float beam_angle = angle_min;
            for (int loop2 = 0; loop2 < 2; loop2++)
                for (int loop = 0; loop < nb_beams;
                     loop++, beam_angle += angle_inc) {
                    // ROS_INFO("hit[%i]: (%f, %f) -> (%f, %f)", loop,
                    // range[loop], beam_angle*180/M_PI, current_scan[loop].x,
                    // current_scan[loop].y);
                    if ((fabs(current_scan[loop][loop2].y) < robair_size) &&
                        (fabs(closest_obstacle.x) >
                         fabs(current_scan[loop][loop2].x)) &&
                        (current_scan[loop][loop2].x >= 0)) {
                        closest_obstacle  = current_scan[loop][loop2];
                        obstacle_detected = true;
                        // ROS_INFO("closest obstacle: (%f; %f)",
                        // closest_obstacle.x, closest_obstacle.y); getchar();
                    }
                }

            if ((obstacle_detected) /*&& ( distancePoints(closest_obstacle, previous_closest_obstacle) > 0.05 )*/)
            {
                pub_closest_obstacle.publish(closest_obstacle);

                nb_pts = 0;
                // closest obstacle is red
                display[nb_pts] = closest_obstacle;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                nb_pts++;
                populateMarkerTopic();
            }
            if (distancePoints(closest_obstacle, previous_closest_obstacle) >
                0.05) {
                ROS_INFO("closest obstacle: (%f; %f)", closest_obstacle.x,
                         closest_obstacle.y);

                previous_closest_obstacle.x = closest_obstacle.x;
                previous_closest_obstacle.y = closest_obstacle.y;
            }
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

        // store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams  = ((-1 * angle_min) + angle_max) / angle_inc;

        // store the range and the coordinates in cartesian framework of each
        // hit
        float beam_angle = angle_min;
        for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
            if ((scan->ranges[loop] < range_max) &&
                (scan->ranges[loop] > range_min))
                range[loop][0] = scan->ranges[loop];
            else
                range[loop][0] = range_max;

            // transform the scan in cartesian framewrok
            current_scan[loop][0].x = range[loop][0] * cos(beam_angle);
            current_scan[loop][0].y = range[loop][0] * sin(beam_angle);
            current_scan[loop][0].z = 0.0;
            // ROS_INFO("laser[%i]: (%f, %f) -> (%f, %f)", loop, range[loop],
            // beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);
        }

    }  // scanCallback

    void scanCallback2(const sensor_msgs::LaserScan::ConstPtr &scan) {
        init_laser2 = true;
        // store the important data related to laserscanner
        range_min = scan->range_min;
        range_max = scan->range_max;
        angle_min = scan->angle_min;
        angle_max = scan->angle_max;
        angle_inc = scan->angle_increment;
        nb_beams  = ((-1 * angle_min) + angle_max) / angle_inc;

        // store the range and the coordinates in cartesian framework of each
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
