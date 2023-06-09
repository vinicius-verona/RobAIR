// moving person detector using lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"

//used for clustering
#define cluster_threshold 0.2 //threshold for clustering

//used for detection of motion
#define detection_threshold 0.2 //threshold for motion detection
#define dynamic_threshold 75 //to decide if a cluster is static or dynamic

//used for detection of moving legs
#define leg_size_min 0.1
#define leg_size_max 0.25

//used for detection of moving person
#define legs_distance_max 0.7

using namespace std;

class moving_person_detector {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_moving_person_detector;
    ros::Publisher pub_moving_person_detector_marker;

    // to store, process and display laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    float background[1000];
    bool dynamic[1000];

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];// to store the size of each cluster
    geometry_msgs::Point cluster_center[1000];// to store the center of gravity of each cluster
    int cluster_dynamic[1000];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000], cluster_end[1000];

    //to perform detection of moving legs and to store them
    int nb_moving_legs_detected;
    geometry_msgs::Point moving_leg_detected[1000];

    //to perform detection of a moving person and store it
    int nb_moving_persons_detected;
    geometry_msgs::Point moving_person_detected;

    //to store the goal to reach that we will be published
    geometry_msgs::Point goal_to_reach;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

    //to check if the robot is moving or not
    bool previous_robot_moving;
    bool current_robot_moving;

    bool init_laser;//to check if new data of laser is available or not
    bool init_robot;//to check if new data of robot_moving is available or not

    bool display_laser;
    bool display_robot;

public:

moving_person_detector() {

    sub_scan = n.subscribe("scan", 1, &moving_person_detector::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &moving_person_detector::robot_movingCallback, this);

    pub_moving_person_detector_marker = n.advertise<visualization_msgs::Marker>("moving_person_detector", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    pub_moving_person_detector = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    current_robot_moving = true;
    init_laser = false;
    init_robot = false;
    display_laser = false;
    display_robot = false;

    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( init_laser && init_robot ) {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_pts = 0;

        //if the robot is not moving then we can perform moving person detection
        if ( !current_robot_moving ) {

            // if the robot was moving previously and now it is not moving now then we store the background
            if ( previous_robot_moving && !current_robot_moving )
                store_background();

            //we search for moving person in 4 steps
            detect_motion();//to classify each hit of the laser as dynamic or not
            perform_clustering();//to perform clustering
            detect_moving_legs();//to detect moving legs using cluster
            detect_moving_persons();//to detect moving_person using moving legs detected

            //graphical display of the results
            populateMarkerTopic();

        }
        //ROS_INFO("");
    }
    else
        if ( !display_laser && !init_laser ) {
            ROS_INFO("wait for laser data");
            display_laser = true;
        }
        if ( display_laser && init_laser )  {
            ROS_INFO("laser data are ok");
            display_laser = false;
        }
        if ( !display_robot && !init_robot ) {
            ROS_INFO("wait for robot_moving_node");
            display_robot = true;
        }
        if ( display_robot && init_robot ) {
            ROS_INFO("robot_moving_node is ok");
            display_robot = false;
        }

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = range[loop];

}//init_background

void detect_motion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("detecting motion");

    for (int loop=0; loop<nb_beams; loop++ ) {
        if ( ( ( background[loop] - range[loop] ) > detection_threshold ) || //we r getting closer to the robot
               ( ( ( range[loop] - background[loop] ) > detection_threshold ) && ( ( range[loop] - background[loop] ) < 2*detection_threshold ) ) )
             dynamic[loop] = 1;
        else
            dynamic[loop] = 0;

        if ( dynamic[loop] ) {
            //display in blue of hits that are dynamic
            display[nb_pts].x = current_scan[loop].x;
            display[nb_pts].y = current_scan[loop].y;
            display[nb_pts].z = current_scan[loop].z;

            colors[nb_pts].r = 0;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;
        }
    }

}//detect_motion

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is higher than a threshold
//else we start a new cluster

    ROS_INFO("performing clustering");

    nb_cluster = 0;

    cluster_start[0] = 0;// the first hit is the start of the first cluster
    cluster[0] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    for( int loop=1; loop<nb_beams; loop++ )
        if ( distancePoints(current_scan[loop-1], current_scan[loop]) < cluster_threshold ) {
            cluster[loop] = nb_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;
        }
        else {
            cluster_end[nb_cluster] = loop-1;
            cluster_dynamic[nb_cluster] = nb_dynamic*100 / (cluster_end[nb_cluster]-cluster_start[nb_cluster]+1);
            cluster_size[nb_cluster] = distancePoints(current_scan[cluster_start[nb_cluster]], current_scan[cluster_end[nb_cluster]]);
            cluster_center[nb_cluster].x = (current_scan[cluster_start[nb_cluster]].x+current_scan[cluster_end[nb_cluster]].x)/2;
            cluster_center[nb_cluster].y = (current_scan[cluster_start[nb_cluster]].y+current_scan[cluster_end[nb_cluster]].y)/2;
            cluster_center[nb_cluster].z = (current_scan[cluster_start[nb_cluster]].z+current_scan[cluster_end[nb_cluster]].z)/2;

            ROS_INFO("cluster[%i]: (%f, %f) -> (%f, %f), size: %f, dynamic: %i", nb_cluster, current_scan[cluster_start[nb_cluster]].x, current_scan[cluster_start[nb_cluster]].y, current_scan[cluster_end[nb_cluster]].x, current_scan[cluster_end[nb_cluster]].y, cluster_size[nb_cluster], cluster_dynamic[nb_cluster]);

            /*//display of the current cluster
            nb_pts = 0;
            for(int loop2=cluster_start[nb_cluster]; loop2<=cluster_end[nb_cluster]; loop2++) {
                // clusters are white
                display[nb_pts].x = current_scan[loop2].x;
                display[nb_pts].y = current_scan[loop2].y;
                display[nb_pts].z = current_scan[loop2].z;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }
            populateMarkerTopic();
            getchar();*/

            nb_dynamic = 0;
            nb_cluster++;
            cluster_start[nb_cluster] = loop;
            cluster[loop] = nb_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;
        }

    cluster_end[nb_cluster] = nb_beams-1;
    cluster_dynamic[nb_cluster] = nb_dynamic*100 / (cluster_end[nb_cluster]-cluster_start[nb_cluster]+1);
    cluster_size[nb_cluster] = distancePoints(current_scan[cluster_start[nb_cluster]], current_scan[cluster_end[nb_cluster]]);
    cluster_center[nb_cluster].x = (current_scan[cluster_start[nb_cluster]].x+current_scan[cluster_end[nb_cluster]].x)/2;
    cluster_center[nb_cluster].y = (current_scan[cluster_start[nb_cluster]].y+current_scan[cluster_end[nb_cluster]].y)/2;
    cluster_center[nb_cluster].z = (current_scan[cluster_start[nb_cluster]].z+current_scan[cluster_end[nb_cluster]].z)/2;

    ROS_INFO("cluster[%i]: (%f, %f) -> (%f, %f), size: %f, dynamic: %i", nb_cluster, current_scan[cluster_start[nb_cluster]].x, current_scan[cluster_start[nb_cluster]].y, current_scan[cluster_end[nb_cluster]].x, current_scan[cluster_end[nb_cluster]].y, cluster_size[nb_cluster], cluster_dynamic[nb_cluster]);

    /*//display of the current cluster
    nb_pts = 0;
    for(int loop2=cluster_start[nb_cluster]; loop2<=cluster_end[nb_cluster]; loop2++) {
        // clusters are white
        display[nb_pts].x = current_scan[loop2].x;
        display[nb_pts].y = current_scan[loop2].y;
        display[nb_pts].z = current_scan[loop2].z;

        colors[nb_pts].r = 1;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 1;
        colors[nb_pts].a = 1.0;

        nb_pts++;
    }
    populateMarkerTopic();
    getchar();*/

    nb_cluster++;

}//perform_clustering

// DETECTION OF MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_moving_legs() {

    ROS_INFO("detecting moving legs");
    nb_moving_legs_detected = 0;

    for (int loop=0; loop<nb_cluster; loop++)
        if ( ( cluster_size[loop]<leg_size_max ) && ( cluster_size[loop]>leg_size_min ) && ( cluster_dynamic[loop]>dynamic_threshold ) ) {
            moving_leg_detected[nb_moving_legs_detected].x = cluster_center[loop].x;
            moving_leg_detected[nb_moving_legs_detected].y = cluster_center[loop].y;
            moving_leg_detected[nb_moving_legs_detected].z = cluster_center[loop].z;

            ROS_INFO("moving leg detected[%i]: cluster[%i]", nb_moving_legs_detected, loop);

            for(int loop2=cluster_start[loop]; loop2<=cluster_end[loop]; loop2++) {
                // moving legs are white
                display[nb_pts].x = current_scan[loop2].x;
                display[nb_pts].y = current_scan[loop2].y;
                display[nb_pts].z = current_scan[loop2].z;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }

            nb_moving_legs_detected++;
        }

    if ( nb_moving_legs_detected )
        ROS_INFO("%d moving legs have been detected.\n", nb_moving_legs_detected);

}//detect_moving_legs

void detect_moving_persons() {

    ROS_INFO("detecting moving persons");
    nb_moving_persons_detected = 0;

    for (int loop_leg1=0; loop_leg1<nb_moving_legs_detected; loop_leg1++)
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_moving_legs_detected; loop_leg2++)
            if ( distancePoints(moving_leg_detected[loop_leg1], moving_leg_detected[loop_leg2]) < legs_distance_max ) {
                moving_person_detected.x = ( moving_leg_detected[loop_leg1].x + moving_leg_detected[loop_leg2].x ) / 2;
                moving_person_detected.y = ( moving_leg_detected[loop_leg1].y + moving_leg_detected[loop_leg2].y ) / 2;
                moving_person_detected.z = ( moving_leg_detected[loop_leg1].z + moving_leg_detected[loop_leg2].z ) / 2;

                ROS_INFO("moving person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1, loop_leg2, moving_person_detected.x, moving_person_detected.y);

                // moving person is green
                display[nb_pts].x = moving_person_detected.x;
                display[nb_pts].y = moving_person_detected.y;
                display[nb_pts].z = moving_person_detected.z;

                colors[nb_pts].r = 0;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;

                nb_pts++;

                //update of the goal
                goal_to_reach.x = moving_person_detected.x;
                goal_to_reach.y = moving_person_detected.y;
                goal_to_reach.z = moving_person_detected.z;

                nb_moving_persons_detected++;
            }

    if ( nb_moving_persons_detected ) {
        ROS_INFO("%d moving person have been detected.\n", nb_moving_persons_detected);
        pub_moving_person_detector.publish(goal_to_reach);
        //getchar();
    }

}//detect_moving_person

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    ROS_INFO("New data of laser received");
    ROS_INFO("range_min, range_max: %f, %f", range_min, range_max);
    ROS_INFO("angle_min: %f", angle_min*180/M_PI);
    ROS_INFO("angle_max: %f", angle_max*180/M_PI);
    ROS_INFO("angle_increment: %f", angle_inc*180/M_PI);
    ROS_INFO("number_of_beams: %d", nb_beams);

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop] = scan->ranges[loop];
        else
            range[loop] = range_max;

        //transform the scan in cartesian framework
        current_scan[loop].x = range[loop] * cos(beam_angle);
        current_scan[loop].y = range[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
    ROS_INFO("New data of robot_moving received");
    previous_robot_moving = current_robot_moving;
    current_robot_moving = state->data;

    if ( current_robot_moving )
        ROS_INFO("robot is moving");
    else
        ROS_INFO("robot is not moving");

}//robot_movingCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_moving_person_detector_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
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

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_moving_person_detector_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "moving_person_detector");

    moving_person_detector bsObject;

    ros::spin();

    return 0;
}
