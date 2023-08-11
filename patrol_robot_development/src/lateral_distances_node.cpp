// Signal handling
#include <patrol_robot_development/LateralDistancesMsg.h>
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

using namespace std;

float clamp(float orientation) {
    while (orientation > M_PI)
        orientation -= 2 * M_PI;

    while (orientation < -M_PI)
        orientation += 2 * M_PI;

    return orientation;
}

class lateral_distances {
private:
    ros::NodeHandle n;

    // communication with laser_scanner
    ros::Subscriber sub_scan;
    ros::Subscriber sub_scan2;

    // communication with action
    ros::Publisher pub_closest_obstacles;
    ros::Publisher pub_closest_obstacles_marker;

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

    patrol_robot_development::LateralDistancesMsg lateral_distances_msg;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

public:
    lateral_distances() {
        frame_origin.x = 0;
        frame_origin.y = 0;

        // Communication with laser scanner
        sub_scan  = n.subscribe("scan", 1, &lateral_distances::scanCallback, this);
        sub_scan2 = n.subscribe("scan2", 1, &lateral_distances::scanCallback2, this);

        // Communication with translation_action
        pub_closest_obstacles = n.advertise<patrol_robot_development::LateralDistancesMsg>("lateral_distances", 1);
        pub_closest_obstacles_marker =
            n.advertise<visualization_msgs::Marker>("lateral_distance_marker",
                                                    1);  // Preparing a topic to publish our results. This will be used
                                                         // by the visualization tool rviz
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
            r.sleep();        // we wait if the processing (ie, callback+update) has
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
                for (int loop = 0; loop < nb_beams; loop++, beam_angle += angle_inc) {
                    // Looking for the lateral of the robot
                    // we assume the robot has a radius of 75cm
                    if (fabs(current_scan[loop][loop2].x) > x_axis_limit)
                        continue;

                    // Check the closest obstacle in the left (+90 degs)
                    if (clamp(beam_angle) >= left_angle_start && (current_scan[loop][loop2].y > robair_size) &&
                        (fabs(lt_closest_obstacle.y) > fabs(current_scan[loop][loop2].y))) {
                        lt_closest_obstacle  = current_scan[loop][loop2];
                        lt_obstacle_detected = true;
                    }

                    // Check the closest obstacle in the right (-90 degs)
                    if (clamp(beam_angle) <= right_angle_start && (current_scan[loop][loop2].y < -robair_size) &&
                        (fabs(rt_closest_obstacle.y) > fabs(current_scan[loop][loop2].y))) {
                        rt_closest_obstacle  = current_scan[loop][loop2];
                        rt_obstacle_detected = true;
                    }
                }

            if (lt_obstacle_detected || rt_obstacle_detected) {
                lateral_distances_msg.lt_obstacle_point    = lt_closest_obstacle;
                lateral_distances_msg.rt_obstacle_point    = rt_closest_obstacle;
                lateral_distances_msg.lt_obstacle_distance = distancePoints(frame_origin, lt_closest_obstacle);
                lateral_distances_msg.rt_obstacle_distance = distancePoints(frame_origin, rt_closest_obstacle);
                pub_closest_obstacles.publish(lateral_distances_msg);

                // closest obstacle left is green (if found)
                nb_pts = 0;
                if (lt_obstacle_detected) {
                    display[nb_pts] = lt_closest_obstacle;

                    colors[nb_pts].r = 0;
                    colors[nb_pts].g = 1;
                    colors[nb_pts].b = 0;
                    colors[nb_pts].a = 1.0;
                }
                nb_pts++;

                // closest obstacle right is pink (if found)
                if (rt_obstacle_detected) {
                    display[nb_pts] = rt_closest_obstacle;

                    colors[nb_pts].r = 1;
                    colors[nb_pts].g = 0;
                    colors[nb_pts].b = 1;
                    colors[nb_pts].a = 1.0;
                }
                nb_pts++;
                populateMarkerTopic();
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
            if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
                range[loop][0] = scan->ranges[loop];
            else
                range[loop][0] = range_max;

            // Transform the scan in cartesian framewrok
            current_scan[loop][0].x = range[loop][0] * cos(beam_angle);
            current_scan[loop][0].y = range[loop][0] * sin(beam_angle);
            current_scan[loop][0].z = 0.0;
        }

        // ROS_WARN("range_min (scan1): %f\n range_max (scan1): %f\n angle_min "
        //          "(scan1): %f\n angle_max (scan1): "
        //          "%f\n angle_increment (scan1): %f,\n\n",
        //          scan->range_min, scan->range_max, scan->angle_min,
        //          scan->angle_max, scan->angle_increment);

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
            if ((scan->ranges[loop] < range_max) && (scan->ranges[loop] > range_min))
                range[loop][1] = scan->ranges[loop];
            else
                range[loop][1] = range_max /*+ 0.2*/;

            // transform the scan in cartesian framewrok
            current_scan[loop][1].x = transform_laser.x + range[loop][1] * cos(beam_angle);
            current_scan[loop][1].y = range[loop][1] * sin(beam_angle);
            current_scan[loop][1].z = transform_laser.z;
        }

        // ROS_WARN("range_min (scan2): %f\n range_max (scan2): %f\n angle_min "
        //          "(scan2): %f\n angle_max (scan2): "
        //          "%f\n angle_increment (scan2): %f,\n\n",
        //          scan->range_min, scan->range_max, scan->angle_min,
        //          scan->angle_max, scan->angle_increment);
    }  // scanCallback2

    void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr &obs) {
        // init_obstacle    = true;
        // closest_obstacle = *obs;
    }  // closest_obstacleCallback

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

        pub_closest_obstacles_marker.publish(references);
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

        pub_closest_obstacles_marker.publish(marker);
        populateMarkerReference();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lateral_distances");
    ros::NodeHandle n;

    ROS_INFO("(lateral_distances) PARAMETERS");

    ros::param::get("/obstacle_avoidance_node/robot_size", robair_size);
    ROS_INFO("(lateral_distances) robot_size: %f", robair_size);

    lateral_distances bsObject;

    ros::spin();

    return 0;
}
