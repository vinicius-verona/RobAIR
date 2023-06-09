// person detector using 2 lidar data
// written by O. Aycard

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

//IMPORTANT
//IMPORTANT
//IMPORTANT
//IMPORTANT
//to switch from offline data to online data
// data_from_bag is true for offline data, its false for real data
bool data_from_bag = true;

#define unassociated -1
#define left 0
#define right 1
#define chest 0

#define detection_threshold 0.1 //threshold for motion detection
#define dynamic_threshold 75 //to decide if a cluster is static or dynamic

//threshold for clustering
#define cluster_threshold 0.2

//used for detection of leg
#define leg_size_min 0.05
#define leg_size_max 0.2
#define legs_distance_min 0
#define legs_distance_max 0.7

//used for detection of chest
#define chest_size_min 0.3
#define chest_size_max 0.8

#define distance_level 0.6

//used for frequency
#define frequency_init 5
#define frequency_max 25

//used for uncertainty of leg
#define uncertainty_min_leg 1
#define uncertainty_max_leg 1
#define uncertainty_inc_leg 0.05

//used for uncertainty of chest
#define uncertainty_min_chest 0.5
#define uncertainty_max_chest 0.5
#define uncertainty_inc_chest 0.05

using namespace std;

class person_detector_tracker
{

private:
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_scan2;

    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_person_detector_tracker_marker;

    // communication with odometry
    ros::Subscriber sub_odometry;
    geometry_msgs::Point current_odom;
    geometry_msgs::Point previous_odom;
    bool init_odom;
    geometry_msgs::Point robot_position;
    float robot_orientation;

    ros::Publisher pub_moving_left_leg_detector;
    ros::Publisher pub_moving_right_leg_detector;
    ros::Publisher pub_moving_person2_detector;
    ros::Publisher pub_stamped_person_tracker;
    ros::Publisher pub_person_tracker;

    // to store, process and display both laserdata
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float range[1000][2];
    geometry_msgs::Point current_scan[1000][2];
    geometry_msgs::Point laser_scan[1000][2];
    bool init_laser, init_laser2;
    geometry_msgs::Point transform_laser;

    //to perform detection of motion
    bool stored_background;
    float background[1000][2];
    bool dynamic[1000][2];
    bool init_robot;
    bool current_robot_moving;

    //to perform clustering
    int nb_cluster[2];// number of cluster
    int cluster[1000][2]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000][2];// to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000][2];// to store the middle of each cluster
    float cluster_dynamic[1000][2];// to store the percentage of the cluster that is dynamic
    int cluster_start[1000][2], cluster_end[1000][2];
    bool local_minimum[1000][2];

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    //to perform detection of chest and to store them
    int nb_chests_detected;
    geometry_msgs::Point chest_detected[1000];
    int chest_cluster[1000];//to store the cluster corresponding to a chest
    bool chest_dynamic[1000];//to know if a chest is dynamic or not

    //to perform tracking of legs
    bool person_tracked;
    int nb_legs_tracked;
    geometry_msgs::Point leg_tracked[1000];
    float leg_uncertainty[1000];
    int leg_frequency[1000];

    //to perform association of legs
    int best_leg_tracked[1000];
    bool best_leg_detected[1000];
    float best_leg_uncertainty;

    //to perform tracking of chests
    int nb_chests_tracked;
    geometry_msgs::Point chest_tracked[1000];
    float chest_uncertainty[1000];
    int chest_frequency[1000];
    geometry_msgs::Point chest_left, chest_right;

    //to perform association of chests
    int best_chest_tracked[1000];
    bool best_chest_detected[1000];
    float best_chest_uncertainty;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool laser_display;
    bool robot_display;
    bool first;

	geometry_msgs::Point previous_position_tracked;

public:

person_detector_tracker()
{

    pub_person_detector_tracker_marker = n.advertise<visualization_msgs::Marker>("one_person_detector_tracker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    sub_scan = n.subscribe("scan", 1, &person_detector_tracker::scanCallback, this);
    sub_scan2 = n.subscribe("scan2", 1, &person_detector_tracker::scanCallback2, this);

    sub_robot_moving = n.subscribe("robot_moving", 1, &person_detector_tracker::robot_movingCallback, this);

    // communication with decision/control
    //pub_stamped_person_tracker = n.advertise<geometry_msgs::PointStamped>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.
//    pub_person_tracker = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.
    pub_person_tracker = n.advertise<geometry_msgs::Point>("person_position", 1);     // Preparing a topic to publish the goal to reach.

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &person_detector_tracker::odomCallback, this);
    init_odom = false;

    init_laser = false;
    init_laser2 = false;
    init_robot = false;
    stored_background = false;

    person_tracked = false;    

    tf::StampedTransform transform;
    tf::TransformListener listener;

    transform_laser.x = -.35;
    transform_laser.y = 0;
    transform_laser.z = 1.2;

	previous_position_tracked.x = 0;
	previous_position_tracked.y = 0;

    first = true;

    /*try {
        listener.waitForTransform("/laser", "/laser2", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/laser", "/laser2", ros::Time(0), transform);
        transform_laser.x = transform.getOrigin().x();
        transform_laser.y = transform.getOrigin().y();
        transform_laser.z = transform.getOrigin().z();
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }*/
    ROS_INFO("/laser2 -> /laser: %f, %f, %f", transform_laser.x, transform_laser.y, transform_laser.z);

    laser_display = false;
    robot_display = false;

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }

}

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update()
{

    //ROS_INFO("%i, %i, %i, %i", data_from_bag, init_laser, init_laser2, init_robot);
    if ( /*data_from_bag &&*/ init_laser && init_laser2 && init_robot && init_odom ) {

        if ( data_from_bag )
        {
            init_laser = false;
            init_laser2 = false;
            init_odom = false;
            init_robot = false;
        }

        nb_pts = 0;

        ROS_INFO(" ");
        if ( person_tracked )
            first = false;
        if ( first && data_from_bag )
            current_robot_moving = false;
        ROS_INFO("detecting a moving person");
        //if the robot is not moving then we can perform moving persons detection
        if ( !current_robot_moving ) {
            ROS_INFO("robot is not moving");
            // if the robot was moving previously and now it is not moving now then we store the background
            if ( !stored_background ) {
                store_background(0);
                store_background(1);
                stored_background = true;
            }
        }
        else {
            ROS_INFO("robot is moving");
            stored_background = false;
        }

        detect_motion(0);   
        perform_clustering(0);
        detect_legs();

        detect_motion(1);
        perform_clustering(1);
        detect_chests();

        if ( !person_tracked )
            detect_moving_persons();
        else {
            detect_2legs();
            track_person();
        }

        populateMarkerTopic();
    }
    else {
        if ( !laser_display ) {
            laser_display = true;
            ROS_INFO("wait for /scan");
        }
        if ( !robot_display ) {
            robot_display = true;
            ROS_INFO("wait for /robot_moving");
        }
    }

}// update

// DETECT MOTION FOR BOTH LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background(int laser) {
// store all the hits of the laser in the background table

    ROS_INFO("store_background for %i laser", laser);

    for (int loop=0; loop<nb_beams; loop++)
        background[loop][laser] = range[loop][laser];

}//init_background

void display_background(int laser) {

    ROS_INFO("display background: %i", laser);
    float r = 1;
    float g = 1;
    float b = 1;

    if ( laser ) {
        r = 0;
        g = 0;
    }

    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        display[nb_pts].x = background[loop][laser] * cos(beam_angle);
        display[nb_pts].y = background[loop][laser] * sin(beam_angle);
        display[nb_pts].z = 0;

        colors[nb_pts].r = r;
        colors[nb_pts].g = g;
        colors[nb_pts].b = b;
        colors[nb_pts].a = 1.0;

        nb_pts++;
    }

}//display_background

void detect_motion(int laser) {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("detect_motion for %i laser", laser);
    //nb_pts = 0;
    for (int loop=0 ; loop<nb_beams; loop++ )
        if ( ( !current_robot_moving ) &&
             //( fabs(background[loop][laser]-range[loop][laser]) > detection_threshold ) ) {
             ( ( ( background[loop][laser] - range[loop][laser] ) > detection_threshold ) || //we r getting closer to the robot
             ( ( ( range[loop][laser] - background[loop][laser] ) > detection_threshold ) && ( ( range[loop][laser] - background[loop][laser] ) < 2*detection_threshold ) ) ) ) {
            dynamic[loop][laser] = true;
      /*      display[nb_pts].x = current_scan[loop][laser].x;
            display[nb_pts].y = current_scan[loop][laser].y;
            display[nb_pts].z = 0;

            colors[nb_pts].r = 1;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 1;
            colors[nb_pts].a = 1.0;

            nb_pts++;*/
        }
        else
            dynamic[loop][laser] = false;

    ROS_INFO("%i points are dynamic", nb_pts);
    //populateMarkerTopic();
    //getchar();

}//detect_motion

// CLUSTERING FOR BOTH LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering(int laser) {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit of the laser and the current one is higher than a threshold
//else we start a new cluster

    ROS_INFO("performing clustering for laser %i", laser);

    nb_cluster[laser] = 0;

    cluster_start[0][laser] = 0;// the first hit is the start of the first cluster
    cluster[0][laser] = 0;// the first hit belongs to the first cluster
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    for( int loop=1; loop<nb_beams; loop++ )
        if ( distancePoints(current_scan[loop-1][laser], current_scan[loop][laser]) < cluster_threshold ) {
            cluster[loop][laser] = nb_cluster[laser];
            if ( dynamic[loop][laser] )
                nb_dynamic++;
        }
        else {

            int current_cluster = nb_cluster[laser];//easier to read
            cluster_end[current_cluster][laser] = loop-1;

            int current_start = cluster_start[current_cluster][laser];
            int current_end = cluster_end[current_cluster][laser];
            cluster_dynamic[current_cluster][laser] = nb_dynamic*100 / (current_end-current_start+1);
            cluster_size[current_cluster][laser] = distancePoints(current_scan[current_start][laser], current_scan[current_end][laser]);
            cluster_middle[current_cluster][laser].x = (current_scan[current_start][laser].x+current_scan[current_end][laser].x)/2;
            cluster_middle[current_cluster][laser].y = (current_scan[current_start][laser].y+current_scan[current_end][laser].y)/2;
            cluster_middle[current_cluster][laser].z = (current_scan[current_start][laser].z+current_scan[current_end][laser].z)/2;

            ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %f", current_cluster,
                                                                                 cluster_middle[current_cluster][laser].x,
                                                                                 cluster_middle[current_cluster][laser].y,
                                                                                 current_start,
                                                                                 current_scan[current_start][laser].x,
                                                                                 current_scan[current_start][laser].y,
                                                                                 current_end,
                                                                                 current_scan[current_end][laser].x,
                                                                                 current_scan[current_end][laser].y,
                                                                                 cluster_size[current_cluster][laser],
                                                                                 nb_dynamic,
                                                                                 cluster_dynamic[current_cluster][laser]);

            /*//display of the current cluster
            nb_pts = 0;
            for(int loop2=current_start; loop2<=current_end; loop2++) {
                // clusters are white
                display[nb_pts].x = current_scan[loop2][laser].x;
                display[nb_pts].y = current_scan[loop2][laser].y;
                display[nb_pts].z = current_scan[loop2][laser].z;

                colors[nb_pts].r = 1;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;

                nb_pts++;
            }
            populateMarkerTopic();
            getchar();*/

            nb_dynamic = 0;
            nb_cluster[laser]++;
            current_cluster++;

            cluster_start[current_cluster][laser] = loop;
            cluster[loop][laser] = current_cluster;
            if ( dynamic[loop] )
                nb_dynamic++;
        }

    int current_cluster = nb_cluster[laser];//easier to read
    int current_start = cluster_start[current_cluster][laser];
    cluster_end[current_cluster][laser] = nb_beams-1;
    int current_end = cluster_end[current_cluster][laser];
    cluster_dynamic[current_cluster][laser] = nb_dynamic*100 / (current_end-current_start+1);
    cluster_size[current_cluster][laser] = distancePoints(current_scan[current_start][laser], current_scan[current_end][laser]);
    cluster_middle[current_cluster][laser].x = (current_scan[current_start][laser].x+current_scan[current_end][laser].x)/2;
    cluster_middle[current_cluster][laser].y = (current_scan[current_start][laser].y+current_scan[current_end][laser].y)/2;
    cluster_middle[current_cluster][laser].z = (current_scan[current_start][laser].z+current_scan[current_end][laser].z)/2;

    ROS_INFO("cluster[%i](%f, %f): [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i, %f", current_cluster,
                                                                         cluster_middle[current_cluster][laser].x,
                                                                         cluster_middle[current_cluster][laser].y,
                                                                         current_start,
                                                                         current_scan[current_start][laser].x,
                                                                         current_scan[current_start][laser].y,
                                                                         current_end,
                                                                         current_scan[current_end][laser].x,
                                                                         current_scan[current_end][laser].y,
                                                                         cluster_size[current_cluster][laser],
                                                                         nb_dynamic,
                                                                         cluster_dynamic[current_cluster][laser]);

    /*//display of the current cluster
    nb_pts = 0;
    for(int loop2=cluster_start[current_cluster]; loop2<=cluster_end[current_cluster]; loop2++) {
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

    nb_cluster[laser]++;

}//perfor_clustering

void search_local_minimum(int laser) {

    geometry_msgs::Point origin;
    origin.x = 0;
    origin.y = 0;

    for(int loop=0; loop<nb_cluster[laser]; loop++) {
        if ( loop == 0 )
            local_minimum[loop][laser] = ( distancePoints(origin, current_scan[cluster_end[loop][laser]][laser]) < distancePoints(origin, current_scan[cluster_start[loop+1][laser]][laser]) );
        else
            if ( loop == nb_cluster[laser]-1 )
                local_minimum[loop][laser] = ( distancePoints(origin, current_scan[cluster_start[loop][laser]][laser]) < distancePoints(origin, current_scan[cluster_end[loop-1][laser]][laser]) );
            else
                local_minimum[loop][laser] = ( distancePoints(origin, current_scan[cluster_end[loop][laser]][laser]) < distancePoints(origin, current_scan[cluster_start[loop+1][laser]][laser]) ) &&
                                             ( distancePoints(origin, current_scan[cluster_start[loop][laser]][laser]) < distancePoints(origin, current_scan[cluster_end[loop-1][laser]][laser]) );
    }

}//local_minimum

void detect_legs() {

    ROS_INFO("detect_legs with 0th laser");
    nb_legs_detected = 0;

    for (int loop=0; loop<nb_cluster[0]; loop++)
        if ( ( cluster_size[loop][0]<leg_size_max ) && ( cluster_size[loop][0]>leg_size_min ) ) {
            leg_detected[nb_legs_detected] = cluster_middle[loop][0];
            leg_dynamic[nb_legs_detected] = ( cluster_dynamic[loop][0] > dynamic_threshold );// && ( cluster_size[loop][0]>leg_size_min );
            float dist_max = 0;

            if ( leg_dynamic[nb_legs_detected] )
                ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %f, local_minimum: %i", nb_legs_detected,
                                                                                                                     loop,
                                                                                                                     leg_detected[nb_legs_detected].x,
                                                                                                                     leg_detected[nb_legs_detected].y,
                                                                                                                     cluster_size[loop][0],
                                                                                                                     cluster_dynamic[loop][0],
                                                                                                                     local_minimum[loop][0]);
            else
                ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %f, local_minimum: %i", nb_legs_detected,
                                                                                                                     loop,
                                                                                                                     leg_detected[nb_legs_detected].x,
                                                                                                                     leg_detected[nb_legs_detected].y,
                                                                                                                     cluster_size[loop][0],
                                                                                                                     cluster_dynamic[loop][0],
                                                                                                                     local_minimum[loop][0]);
            leg_cluster[nb_legs_detected] = loop;
            //nb_pts = 0;
            if ( leg_dynamic[nb_legs_detected] || person_tracked )
                for(int loop2=0; loop2<nb_beams; loop2++)
                    if ( cluster[loop2][0] == loop ) {

                        // to compute the compacity of the leg: NOT USED AT THE MOMENT
                        if ( cluster[loop2+1][0] == loop ) {
                            float dist_current = distancePoints(current_scan[loop2][0], current_scan[loop2+1][0]);
                            if ( dist_current > dist_max )
                                dist_max = dist_current;
                        }

                        // legs are white
                        /*display[nb_pts].x = current_scan[loop2][0].x;
                        display[nb_pts].y = current_scan[loop2][0].y;
                        display[nb_pts].z = current_scan[loop2][0].z;

                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 1;
                        colors[nb_pts].a = 1.0;

                        nb_pts++;*/
                    }
            nb_legs_detected++;
        }

    if ( nb_legs_detected ) {
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);
    }

}//detect_legs

void detect_2legs() {

    ROS_INFO("detect_2legs with 0th laser");
    //nb_legs_detected = 0;

    for (int loop=0; loop<nb_cluster[0]; loop++)
        if ( ( cluster_size[loop][0]<2*leg_size_max ) && ( cluster_size[loop][0]>leg_size_max ) ) {
            float dist_max = cluster_threshold/2;
            int middle;
            for (int loop_cluster=cluster_start[loop][0]; loop_cluster<cluster_end[loop][0]-1; loop_cluster++)
                if ( distancePoints(current_scan[loop_cluster][0], current_scan[loop_cluster+1][0]) > dist_max ) {
                    middle = loop_cluster;
                    dist_max = distancePoints(current_scan[loop_cluster][0], current_scan[loop_cluster+1][0]);
                }
            if ( dist_max > cluster_threshold/2 ) {
                float size1 = distancePoints(current_scan[cluster_start[loop][0]][0], current_scan[middle][0]);
                float size2 = distancePoints(current_scan[middle+1][0], current_scan[cluster_end[loop][0]][0]);
                if ( ( size1<leg_size_max ) && ( size1>leg_size_min ) ) {
                    leg_detected[nb_legs_detected].x = ( current_scan[cluster_start[loop][0]][0].x + current_scan[middle][0].x)/2;
                    leg_detected[nb_legs_detected].y = ( current_scan[cluster_start[loop][0]][0].y + current_scan[middle][0].y)/2;

                    ROS_INFO("leg found: %i -> cluster = %i, (%f, %f), size: %f", nb_legs_detected,
                                                                                  loop,
                                                                                  leg_detected[nb_legs_detected].x,
                                                                                  leg_detected[nb_legs_detected].y,
                                                                                  size1);
                    nb_legs_detected++;
                }
                if ( ( size2<leg_size_max ) && ( size2>leg_size_min ) ) {
                    leg_detected[nb_legs_detected].x = ( current_scan[cluster_end[loop][0]][0].x + current_scan[middle+1][0].x)/2;
                    leg_detected[nb_legs_detected].y = ( current_scan[cluster_end[loop][0]][0].y + current_scan[middle+1][0].y)/2;

                    ROS_INFO("leg found: %i -> cluster = %i, (%f, %f), size: %f", nb_legs_detected,
                                                                                  loop,
                                                                                  leg_detected[nb_legs_detected].x,
                                                                                  leg_detected[nb_legs_detected].y,
                                                                                  size2);
                    nb_legs_detected++;
                }
            }
        }

    if ( nb_legs_detected ) {
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);
    }

}//detect_2legs

// PROCESSING OF DATA OF THE 2ND LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_chests() {

    ROS_INFO("detect_chests with 1st laser");
    nb_chests_detected = 0;

    for (int loop=0; loop<nb_cluster[1]; loop++)
        if ( ( cluster_size[loop][1] < chest_size_max ) && ( cluster_size[loop][1] > chest_size_min )/*|| person_tracked*/ ) {
            chest_detected[nb_chests_detected] = cluster_middle[loop][1];
            chest_dynamic[nb_chests_detected] = ( cluster_dynamic[loop][1]>dynamic_threshold ) ;

            /*chest_detected2_leg1[nb_chests_detected2].x = cluster_leg1[loop].x;
            chest_detected2_leg1[nb_chests_detected2].y = cluster_leg1[loop].y;

            chest_detected2_leg2[nb_chests_detected2].x = cluster_leg2[loop].x;
            chest_detected2_leg2[nb_chests_detected2].y = cluster_leg2[loop].y;*/

            //ROS_INFO("person detected2: %i -> cluster2: %i, center: (%f, %f), lcenter: (%f, %f), rcenter: (%f, %f), size: %f", nb_chests_detected2, loop, cluster_middle2[loop].x, cluster_middle2[loop].y, cluster_leg1[loop].x, cluster_leg1[loop].y,cluster_leg2[loop].x, cluster_leg2[loop].y, cluster_size2[loop]);
            if ( !chest_dynamic[nb_chests_detected] )
                ROS_INFO("static chest detected: %i -> cluster2: %i, center: (%f, %f), size: %f, dynamic: %f, local_minimum: %i", nb_chests_detected,
                                                                                                                                  loop,
                                                                                                                                  chest_detected[nb_chests_detected].x,
                                                                                                                                  chest_detected[nb_chests_detected].y,
                                                                                                                                  cluster_size[loop][1],
                                                                                                                                  cluster_dynamic[loop][1],
                                                                                                                                   local_minimum[loop][1]);
            else
                ROS_INFO("moving chest detected: %i -> cluster2: %i, center: (%f, %f), size: %f, dynamic: %f, local_minimum: %i", nb_chests_detected,
                                                                                                                                  loop,
                                                                                                                                  chest_detected[nb_chests_detected].x,
                                                                                                                                  chest_detected[nb_chests_detected].y,
                                                                                                                                  cluster_size[loop][1],
                                                                                                                                  cluster_dynamic[loop][1],
                                                                                                                                   local_minimum[loop][1]);                

            chest_cluster[nb_chests_detected] = loop;
            //nb_pts = 0;
            /*if ( chest_dynamic[nb_chests_detected] || person_tracked )
                for(int loop3=0; loop3<nb_beams; loop3++)
                    if ( cluster[loop3][1] == loop ) {

                        // static chests are blue and dynamic chests are purple
                        display[nb_pts].x = current_scan[loop3][1].x;
                        display[nb_pts].y = current_scan[loop3][1].y;
                        display[nb_pts].z = current_scan[loop3][1].z;

                        colors[nb_pts].r = 0;
                        colors[nb_pts].g = 0;
                        colors[nb_pts].b = 1;
                        colors[nb_pts].a = 1.0;
                        nb_pts++;
                    }*/
                //ROS_INFO("person detected2: %i -> cluster2: %i, center: (%f, %f), lcenter: (%f, %f), rcenter: (%f, %f), size: %f", nb_chests_detected2, loop, cluster_middle2[loop].x, cluster_middle2[loop].y, cluster_leg1[loop].x, cluster_leg1[loop].y,cluster_leg2[loop].x, cluster_leg2[loop].y, cluster_size2[loop]);

            nb_chests_detected++;
    }

    if ( nb_chests_detected ) {
        ROS_INFO("%d chest detected with 1st laser.\n", nb_chests_detected);
    }

}//detect_chest

// FUSION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_moving_persons() {

    ROS_INFO("detect_moving_persons");

    for (int loop_left_leg=0; loop_left_leg<nb_legs_detected; loop_left_leg++)
        if ( leg_dynamic[loop_left_leg] )
            for (int loop_right_leg=loop_left_leg+1; loop_right_leg<nb_legs_detected; loop_right_leg++)
                if ( leg_dynamic[loop_right_leg] ) {
                    float two_legs_distance = distancePoints(leg_detected[loop_left_leg], leg_detected[loop_right_leg]);
                    bool person_detected = ( two_legs_distance<legs_distance_max );
                    if ( person_detected )
                        for (int loop_chest=0; loop_chest<nb_chests_detected; loop_chest++)
                            if ( chest_dynamic[loop_chest] ) {
                                float left_chest_distance = distancePoints(leg_detected[loop_left_leg], chest_detected[loop_chest]);
                                float right_chest_distance = distancePoints(leg_detected[loop_right_leg], chest_detected[loop_chest]);

                                person_detected = ( left_chest_distance<distance_level ) && ( right_chest_distance<distance_level );

                                if ( person_detected && !person_tracked ) {
                                    initialize_tracking(loop_left_leg, loop_right_leg, loop_chest);

                                }
                            }
                }

}//detect_persons

void initialize_tracking(int l, int r, int c) {

    ROS_INFO("initialize_tracking");

    person_tracked = true;

    //initialization of left leg (ie, leg_tracked[0])
    nb_legs_tracked = 2;
    leg_tracked[left].x = leg_detected[l].x;
    leg_tracked[left].y = leg_detected[l].y;
    leg_tracked[left].z = 0;

    leg_uncertainty[left] = uncertainty_min_leg;
    leg_frequency[left] = frequency_init;

    //initialization of right leg (ie, leg_tracked[1])
    leg_tracked[right].x = leg_detected[r].x;
    leg_tracked[right].y = leg_detected[r].y;
    leg_tracked[right].z = 0;

    leg_uncertainty[right] = uncertainty_min_leg;
    leg_frequency[right] = frequency_init;

    //initialization of left_right
    /*left_right.x = leg_tracked[0].x-leg_tracked[1].x;
    left_right.y = leg_tracked[0].y-leg_tracked[1].y;*/

    //initialization of chest
    nb_chests_tracked = 1;
    chest_tracked[chest].x = chest_detected[c].x;
    chest_tracked[chest].y = chest_detected[c].y;
    chest_tracked[chest].z = 1.2;

    chest_uncertainty[chest] = uncertainty_min_chest;
    chest_frequency[chest] = frequency_init;

    chest_left.x = leg_tracked[left].x-chest_tracked[chest].x;
    chest_left.y = leg_tracked[left].y-chest_tracked[chest].y;

    chest_right.x = leg_tracked[right].x-chest_tracked[chest].x;
    chest_right.y = leg_tracked[right].y-chest_tracked[chest].y;

    ROS_INFO("frequency: %i", frequency_init);
    ROS_INFO("left (%f, %f), %f", leg_tracked[left].x, leg_tracked[left].y, leg_uncertainty[left]);
    ROS_INFO("right (%f, %f), %f", leg_tracked[right].x, leg_tracked[right].y, leg_uncertainty[right]);
    ROS_INFO("chest (%f, %f), %f", chest_tracked[chest].x, chest_tracked[chest].y, chest_uncertainty[chest]);
    ROS_INFO("chest_left: (%f, %f)", chest_left.x, chest_left.y);
    ROS_INFO("chest_right: (%f, %f)", chest_right.x, chest_right.y);

    ROS_INFO("person has been detected");

    geometry_msgs::PointStamped stamped_person_tracked;
    stamped_person_tracked.header.frame_id = "/laser";
    stamped_person_tracked.header.stamp = ros::Time(0);

    geometry_msgs::Point person_tracked;
    person_tracked.x = ( leg_tracked[left].x + leg_tracked[right].x + chest_tracked[chest].x ) / 3;
    person_tracked.y = ( leg_tracked[left].y + leg_tracked[right].y + chest_tracked[chest].y ) / 3;

    stamped_person_tracked.point.x = person_tracked.x;
    stamped_person_tracked.point.y = person_tracked.y;

    display[nb_pts].x = person_tracked.x;
    display[nb_pts].y = person_tracked.y;
    display[nb_pts].z = 0;

    //detection is green
    colors[nb_pts].r = 0.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 0.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    //pub_stamped_person_tracker.publish(stamped_person_tracked);
    pub_person_tracker.publish(person_tracked);

}//initialize_tracking

void track_person() {

    predict_person();
    associate_person();
    estimate_person();

}//track_person

void predict_person() {

    ROS_INFO("predict_person");
    float diff_x = previous_odom.x - current_odom.x;
    float diff_y = previous_odom.y - current_odom.y;
    ROS_INFO("previous_odom(%f, %f) -> current_odom(%f, %f) = t(%f, %f)", previous_odom.x, previous_odom.y, current_odom.x, current_odom.y, diff_x, diff_y);
    if ( diff_x || diff_y ) {
        for ( int loop_leg_tracked=0; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++) {
            leg_tracked[loop_leg_tracked].x += diff_x;
            leg_tracked[loop_leg_tracked].y += diff_y;

        }

        for ( int loop_chest_tracked=0; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++) {
            chest_tracked[loop_chest_tracked].x += diff_x;
            chest_tracked[loop_chest_tracked].y += diff_y;
        }
    }
    float diff_orientation = current_odom.z - previous_odom.z;
    if ( diff_orientation > M_PI )
        diff_orientation -= 2*M_PI;
    if ( diff_orientation < -M_PI )
        diff_orientation += 2*M_PI;

    nb_pts = 0;
    ROS_INFO("previous_orientation(%f) -> current_orientation(%f) = rotation(%f)", previous_odom.z*180/M_PI, current_odom.z*180/M_PI, diff_orientation*180/M_PI);
    if ( diff_orientation ) {
        for ( int loop_leg_tracked=0; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++) {
            geometry_msgs::Point current_leg = leg_tracked[loop_leg_tracked];
            leg_tracked[loop_leg_tracked].x = current_leg.x * cos(diff_orientation) + current_leg.y * sin(diff_orientation);
            leg_tracked[loop_leg_tracked].y = -current_leg.x * sin(diff_orientation) + current_leg.y * cos(diff_orientation);

            /*display[nb_pts] = leg_tracked[loop_leg_tracked];
            colors[nb_pts].r = 1.0f;
            colors[nb_pts].g = 1.0f;
            colors[nb_pts].b = 1.0f;
            colors[nb_pts].a = 1.0;
            nb_pts++;*/
        }

        for ( int loop_chest_tracked=0; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++) {
            geometry_msgs::Point current_chest = chest_tracked[loop_chest_tracked];
            chest_tracked[loop_chest_tracked].x = current_chest.x * cos(diff_orientation) + current_chest.y * sin(diff_orientation);
            chest_tracked[loop_chest_tracked].y = -current_chest.x * sin(diff_orientation) + current_chest.y * cos(diff_orientation);

            /*display[nb_pts] = chest_tracked[loop_chest_tracked];
            colors[nb_pts].r = 1.0f;
            colors[nb_pts].g = 0.0f;
            colors[nb_pts].b = 0.0f;
            colors[nb_pts].a = 1.0;
            nb_pts++;*/

        }
    }

    ROS_INFO("prediction_done");
    /*populateMarkerTopic();
    ROS_INFO("press enter to continue");
    getchar();
    nb_pts = 0;*/

}// predict_person

void associate_person() {

    ROS_INFO("associate_person");
    float best_uncertainty = (nb_legs_tracked+1)*uncertainty_max_leg+nb_chests_tracked*uncertainty_max_chest;
    bool best_found = false;
    ROS_INFO("best_uncertainty: %f", best_uncertainty);
    ROS_INFO("nb_legs_tracked: %i, nb_chests_tracked: %i", nb_legs_tracked, nb_chests_tracked);

    for ( int loop_leg_tracked=0; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++)
        best_leg_tracked[loop_leg_tracked] = unassociated;
    for ( int loop_leg_detected=0; loop_leg_detected<nb_legs_detected; loop_leg_detected++)
        best_leg_detected[loop_leg_detected] = false;

    for ( int loop_chest_tracked=0; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++)
        best_chest_tracked[loop_chest_tracked] = unassociated;
    for ( int loop_chest_detected=0; loop_chest_detected<nb_chests_detected; loop_chest_detected++)
        best_chest_detected[loop_chest_detected] = false;

    //search to associate the left leg
    for (int loop_left=unassociated; loop_left<nb_legs_detected; loop_left++) {
        bool left_associated = true;
        float left_distance = leg_uncertainty[left];
        if ( loop_left != unassociated ) {
            left_distance = distancePoints(leg_detected[loop_left], leg_tracked[left]);
            left_associated = (  left_distance < leg_uncertainty[left] );
        }

        if ( left_associated )
            //search to associate the right leg
            for (int loop_right=unassociated; loop_right<nb_legs_detected; loop_right++) {
                //if ( ( loop_left == unassociated ) || ( loop_right == unassociated ) || ( loop_left != loop_right ) ) {
                bool right_associated = true;
                float right_distance = leg_uncertainty[right];
                if ( loop_right != unassociated ) {
                    right_distance = distancePoints(leg_detected[loop_right], leg_tracked[right]);
                    right_associated = ( right_distance < leg_uncertainty[right] ) && ( loop_left != loop_right );
                }

                if ( right_associated )
                    for (int loop_chest=unassociated;loop_chest<nb_chests_detected; loop_chest++) {
                        bool chest_associated = true;
                        float chest_distance = chest_uncertainty[chest];
                        if ( loop_chest != unassociated ) {
                            chest_distance = distancePoints(chest_detected[loop_chest], chest_tracked[chest]);
                            chest_associated = (  chest_distance < chest_uncertainty[chest] );
                        }

                        if ( chest_associated ) {
                            ROS_INFO("\n");
                            ROS_INFO("trying an association");
                            if ( loop_left == unassociated )
                                ROS_INFO("leg[left]:(%f, %f) is unassociated", leg_tracked[left].x, leg_tracked[left].y);
                            else
                                ROS_INFO("leg[left]:(%f, %f) is associated with detection[%i]: (%f, %f) = %f", leg_tracked[left].x, leg_tracked[left].y, loop_left, leg_detected[loop_left].x, leg_detected[loop_left].y, left_distance);

                            if ( loop_right == unassociated )
                                ROS_INFO("leg[right]:(%f, %f) is unassociated", leg_tracked[right].x, leg_tracked[right].y);
                            else
                                ROS_INFO("leg[right]:(%f, %f) is associated with detection[%i]: (%f, %f) = %f", leg_tracked[right].x, leg_tracked[right].y, loop_right, leg_detected[loop_right].x, leg_detected[loop_right].y, right_distance);

                            if ( loop_chest == unassociated )
                                ROS_INFO("chest[chest]:(%f, %f) is unassociated", chest_tracked[left].x, chest_tracked[left].y);
                            else
                                ROS_INFO("chest[chest]:(%f, %f) is associated with detection[%i]: (%f, %f) = %f", chest_tracked[left].x, chest_tracked[left].y, loop_chest, chest_detected[loop_chest].x, chest_detected[loop_chest].y, chest_distance);

                            geometry_msgs::Point left_leg;
                            if ( loop_left == unassociated )
                                left_leg = leg_tracked[left];
                            else
                                left_leg = leg_detected[loop_left];

                            geometry_msgs::Point right_leg;
                            if ( loop_right == unassociated )
                                right_leg = leg_tracked[right];
                            else
                                right_leg = leg_detected[loop_right];

                            float two_legs_distance = distancePoints(left_leg, right_leg);
                            bool check_two_legs_distance = (  two_legs_distance < legs_distance_max) /*&& ( two_legs_distance > legs_distance_min )*/;

                            if ( check_two_legs_distance )
                                ROS_INFO("two_legs_distance: %f, OK", two_legs_distance);
                            else
                                ROS_INFO("two_legs_distance: %f, not OK", two_legs_distance);

                            geometry_msgs::Point chest_selected;
                            if ( loop_chest == unassociated )
                                chest_selected = chest_tracked[chest];
                            else
                                chest_selected = chest_detected[loop_chest];

                            float chest_left_distance = distancePoints(left_leg, chest_selected);
                            bool check_left_distance = ( chest_left_distance < distance_level );
                            if ( check_left_distance )
                                ROS_INFO("chest_left_distance: %f, OK", chest_left_distance);
                            else
                                ROS_INFO("chest_left_distance: %f, not OK", chest_left_distance);

                            float chest_right_distance = distancePoints(right_leg, chest_selected);
                            bool check_right_distance = ( chest_right_distance < distance_level );
                            if ( check_right_distance )
                                ROS_INFO("chest_right_distance: %f, OK", chest_right_distance);
                            else
                                ROS_INFO("chest_right_distance: %f, not OK", chest_right_distance);

                            int nb_association = 0;
                            if ( loop_left != unassociated )
                                nb_association++;
                            if ( loop_right != unassociated )
                                nb_association++;
                            if ( loop_chest != unassociated )
                                nb_association++;

                            bool association_ok;
                            if ( nb_association < 2 )
                                association_ok = true;
                            else
                                if ( nb_association == 2 ) {
                                    if ( ( loop_left != unassociated ) && ( loop_right != unassociated ) && ( check_two_legs_distance ) )
                                        association_ok = true;
                                    else
                                        if ( ( loop_left != unassociated ) && ( loop_chest != unassociated ) && ( check_left_distance ) )
                                            association_ok =true;
                                        else
                                            if ( ( loop_right != unassociated ) && ( loop_chest != unassociated ) && ( check_right_distance ) )
                                                association_ok =true;
                                }
                                else
                                    association_ok = ( check_two_legs_distance ) && ( check_left_distance ) && ( check_right_distance );

                            association_ok = ( check_two_legs_distance ) && ( check_left_distance ) && ( check_right_distance );

                            if ( association_ok ) {
                                ROS_INFO("association is ok");
                                float current_uncertainty = left_distance+right_distance+chest_distance;

                                int leg_track_association[1000];
                                for ( int loop_leg_tracked=2; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++ )
                                    leg_track_association[loop_leg_tracked] = unassociated;

                                 leg_track_association[left] = loop_left;
                                 leg_track_association[right] = loop_right;

                                 bool leg_detection_association[1000];
                                 for ( int loop_observation=0; loop_observation<nb_legs_detected; loop_observation++ )
                                      leg_detection_association[loop_observation] = false;

                                 if ( loop_left != unassociated )
                                      leg_detection_association[loop_left] = true;
                                 if ( loop_right != unassociated )
                                      leg_detection_association[loop_right] = true;

                                 //chest association
                                 int chest_track_association[1000];
                                 for ( int loop_chest_tracked=1; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++ )
                                      chest_track_association[loop_chest_tracked] = unassociated;

                                  chest_track_association[chest] = loop_chest;

                                  bool chest_detection_association[1000];
                                  for ( int loop_observation=0; loop_observation<nb_chests_detected; loop_observation++ )
                                    chest_detection_association[loop_observation] = false;

                                  if ( loop_chest != unassociated )
                                    chest_detection_association[loop_chest] = true;

                                  bool cond = true;
                                  while ( cond ) {
                                      bool cond_leg = false;
                                      float min_leg_distance = uncertainty_max_leg;
                                      int min_leg_track;
                                      int min_leg_detection;

                                      for (int loop_leg_tracked=2; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++ )
                                          if ( leg_track_association[loop_leg_tracked] == unassociated )
                                              for ( int loop_detection=0; loop_detection<nb_legs_detected; loop_detection++ )
                                                  if ( !leg_detection_association[loop_detection] ) {
                                                      float current_distance = distancePoints(leg_tracked[loop_leg_tracked], leg_detected[loop_detection]);
                                                      if ( current_distance < min_leg_distance ) {
                                                          min_leg_track = loop_leg_tracked;
                                                          min_leg_detection = loop_detection;
                                                          min_leg_distance = current_distance;
                                                          cond_leg = true;
                                                      }
                                                  }

                                      if ( cond_leg ) {
                                        ROS_INFO("leg[%i]: (%f, %f) is associated with detection[%i]: (%f, %f) = %f", min_leg_track, leg_tracked[min_leg_track].x, leg_tracked[min_leg_track].y, min_leg_detection, leg_detected[min_leg_detection].x, leg_detected[min_leg_detection].y, min_leg_distance);
                                        leg_track_association[min_leg_track] = min_leg_detection;
                                        leg_detection_association[min_leg_detection] = true;
                                        current_uncertainty += min_leg_distance;
                                      }

                                      bool cond_chest = false;
                                      float min_chest_distance = uncertainty_max_chest;
                                      int min_chest_track;
                                      int min_chest_detection;

                                      for (int loop_chest_tracked=1; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++ )
                                          if ( chest_track_association[loop_chest_tracked] == unassociated )
                                              for ( int loop_detection=0; loop_detection<nb_chests_detected; loop_detection++ )
                                                  if ( !chest_detection_association[loop_detection] ) {
                                                      float current_distance = distancePoints(chest_tracked[loop_chest_tracked], chest_detected[loop_detection]);
                                                      if ( current_distance < min_chest_distance ) {
                                                          min_chest_track = loop_chest_tracked;
                                                          min_chest_detection = loop_detection;
                                                          min_chest_distance = current_distance;
                                                          cond_chest = true;
                                                      }
                                                }

                                    if ( cond_chest ) {
                                        ROS_INFO("chest[%i]: (%f, %f) is associated with detection[%i]: (%f, %f) = %f", min_chest_track, chest_tracked[min_chest_track].x, chest_tracked[min_chest_track].y, min_chest_detection, chest_detected[min_chest_detection].x, chest_detected[min_chest_detection].y, min_chest_distance);
                                        chest_track_association[min_chest_track] = min_chest_detection;
                                        chest_detection_association[min_chest_detection] = true;
                                        current_uncertainty += min_chest_distance;
                                    }
                                cond = cond_leg || cond_chest;
                                }

                                //to take into account the uncertainty for the track unassociated
                                for (int loop_leg_tracked=2; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++ )
                                    if ( leg_track_association[loop_leg_tracked] == unassociated ) {
                                        current_uncertainty += leg_uncertainty[loop_leg_tracked];
                                        ROS_INFO("leg[%i]:(%f, %f) has been unassociated", loop_leg_tracked, leg_tracked[loop_leg_tracked].x, leg_tracked[loop_leg_tracked].y);
                                    }

                                for (int loop_chest_tracked=2; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++ )
                                    if ( chest_track_association[loop_chest_tracked] == unassociated ) {
                                        current_uncertainty += chest_uncertainty[loop_chest_tracked];
                                        ROS_INFO("chest[%i]:(%f, %f) has been unassociated", loop_chest_tracked, chest_tracked[loop_chest_tracked].x, chest_tracked[loop_chest_tracked].y);
                                    }

                                 ROS_INFO("score: %f", current_uncertainty);
                                 if ( current_uncertainty < best_uncertainty ) {
                                    ROS_INFO("this is the best");
                                    best_found = true;
                                    best_uncertainty = current_uncertainty;
                                    for ( int loop_leg_tracked=0; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++)
                                        best_leg_tracked[loop_leg_tracked] = leg_track_association[loop_leg_tracked];
                                    for ( int loop_detection=0; loop_detection<nb_legs_detected; loop_detection++)
                                        best_leg_detected[loop_detection] = leg_detection_association[loop_detection];
                                    for ( int loop_chest_tracked=0; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++)
                                        best_chest_tracked[loop_chest_tracked] = chest_track_association[loop_chest_tracked];
                                    for ( int loop_detection=0; loop_detection<nb_chests_detected; loop_detection++)
                                        best_chest_detected[loop_detection] = chest_detection_association[loop_detection];
                                 }
                            }
                            else
                                ROS_INFO("association is not ok");
                    }
                }
        }
    }
    if ( !best_found )
        ROS_WARN("no association found");

    ROS_INFO("associate_person done");
    manage_tracks();

}//associate_track()

void manage_tracks() {

    ROS_INFO("\n");
    ROS_INFO("manage tracks");

    manage_tracks_legs();
    manage_tracks_chests();

    ROS_INFO("manage_tracks done");

}

void manage_tracks_legs() {

    ROS_INFO("manage_track_legs");
    for (int loop_leg_tracked=0; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++)
        if ( best_leg_tracked[loop_leg_tracked] != unassociated ) {
            ROS_INFO("leg[%i]:(%f, %f) has been associated with detection[%i]: (%f, %f), frequency = %i", loop_leg_tracked, leg_tracked[loop_leg_tracked].x, leg_tracked[loop_leg_tracked].y, best_leg_tracked[loop_leg_tracked], leg_detected[best_leg_tracked[loop_leg_tracked]].x, leg_detected[best_leg_tracked[loop_leg_tracked]].y, leg_frequency[loop_leg_tracked]);
            leg_tracked[loop_leg_tracked] = leg_detected[best_leg_tracked[loop_leg_tracked]];
            if ( leg_frequency[loop_leg_tracked] < frequency_max )
                leg_frequency[loop_leg_tracked]++;
            leg_uncertainty[loop_leg_tracked] = uncertainty_min_leg;
        }
        else {
            ROS_INFO("leg[%i]:(%f, %f) has been unassociated, frequency = %i", loop_leg_tracked, leg_tracked[loop_leg_tracked].x, leg_tracked[loop_leg_tracked].y, leg_frequency[loop_leg_tracked]);
            //we update the track that have not been detected
            if ( leg_frequency[loop_leg_tracked] > 0 )
                leg_frequency[loop_leg_tracked]--;
            if ( leg_uncertainty[loop_leg_tracked] < uncertainty_max_leg )
                leg_uncertainty[loop_leg_tracked] += uncertainty_inc_leg;
        }

    //we suppress the track with a frequency egals to 0
    int indice = 2;
    for (int loop_leg_tracked=2; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++)
        if ( ( leg_frequency[loop_leg_tracked] ) &&
             ( ( distancePoints(leg_tracked[left], leg_tracked[loop_leg_tracked]) < 2*uncertainty_max_leg ) ||
                   ( distancePoints(leg_tracked[right], leg_tracked[loop_leg_tracked]) < 2*uncertainty_max_leg ) ) ) {
            leg_tracked[indice] = leg_tracked[loop_leg_tracked];
            leg_frequency[indice] = leg_frequency[loop_leg_tracked];
            leg_uncertainty[indice] = leg_uncertainty[loop_leg_tracked];
            indice++;
        }
        else
            ROS_INFO("leg[%i]: (%f, %f) is suppressed", loop_leg_tracked, leg_tracked[loop_leg_tracked].x, leg_tracked[loop_leg_tracked].y);

    nb_legs_tracked = indice;

    //we create new track with the detection not associated
    for (int loop_leg_detected=0; loop_leg_detected<nb_legs_detected; loop_leg_detected++)
        if ( !best_leg_detected[loop_leg_detected] &&
             ( ( distancePoints(leg_tracked[left], leg_detected[loop_leg_detected]) < 2*uncertainty_max_leg ) || ( distancePoints(leg_tracked[right], leg_detected[loop_leg_detected]) < 2*uncertainty_max_leg ) ) ) {
            leg_tracked[nb_legs_tracked] = leg_detected[loop_leg_detected];
            leg_frequency[nb_legs_tracked] = frequency_init;
            leg_uncertainty[nb_legs_tracked] = uncertainty_min_leg;

            ROS_INFO("leg[%i]:(%f, %f) has been created", nb_legs_tracked, leg_detected[loop_leg_detected].x, leg_detected[loop_leg_detected].y);
            nb_legs_tracked++;
        }

    //check track too close from left or right leg
    for (int loop_leg_tracked=2; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++) {
        if ( ( distancePoints(leg_tracked[loop_leg_tracked], leg_tracked[left]) < cluster_threshold ) && ( leg_frequency[left] == 0 ) )
            ROS_WARN("track[%i] is too close of track[left]", loop_leg_tracked);
        if ( ( distancePoints(leg_tracked[loop_leg_tracked], leg_tracked[right]) < cluster_threshold ) && ( leg_frequency[right] == 0 ) )
            ROS_WARN("track[%i] is too close of track[right]", loop_leg_tracked);
    }

    //*nb_pts = 0;
    for (int loop_leg_tracked=2; loop_leg_tracked<nb_legs_tracked; loop_leg_tracked++) {
        /*geometry_msgs::Point laser_leg;
        float range = ( leg_tracked[loop_leg_tracked].x - robot_position.x ) * ( leg_tracked[loop_leg_tracked].x - robot_position.x ) +
                        ( leg_tracked[loop_leg_tracked].y - robot_position.y ) * ( leg_tracked[loop_leg_tracked].y - robot_position.y );
        range = sqrt(range);
        float angle = acos ( ( leg_tracked[loop_leg_tracked].x - robot_position.x ) / range );
        if ( leg_tracked[loop_leg_tracked].y - robot_position.y < 0 )
            angle *= -1;
        angle -= robot_orientation;

        laser_leg.x = range * cos(angle);
        laser_leg.y = range * sin(angle);
        laser_leg.z = 0;

        display[nb_pts] = laser_leg;*/
        display[nb_pts] = leg_tracked[loop_leg_tracked];
        colors[nb_pts].r = 1.0f;
        colors[nb_pts].g = 1.0f;
        colors[nb_pts].b = 0.0f;
        colors[nb_pts].a = 1.0;
        nb_pts++;
    }

    //populateMarkerTopic();*/

    ROS_INFO("%i legs are tracked", nb_legs_tracked);
    ROS_INFO("manage_tracks_legs done\n");

    /*ROS_INFO("enter to continue");
    getchar();*/

}//manage_tracks_legs

void manage_tracks_chests() {

    ROS_INFO("manage_track_chests");
    for (int loop_chest_tracked=0; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++)
        if ( best_chest_tracked[loop_chest_tracked] != unassociated ) {
            ROS_INFO("chest[%i]:(%f, %f) has been associated with detection[%i]: (%f, %f), frequency = %i", loop_chest_tracked, chest_tracked[loop_chest_tracked].x, chest_tracked[loop_chest_tracked].y, best_chest_tracked[loop_chest_tracked], chest_detected[best_chest_tracked[loop_chest_tracked]].x, chest_detected[best_chest_tracked[loop_chest_tracked]].y, chest_frequency[loop_chest_tracked]);
            chest_tracked[loop_chest_tracked] = chest_detected[best_chest_tracked[loop_chest_tracked]];
            if ( chest_frequency[loop_chest_tracked] < frequency_max )
                chest_frequency[loop_chest_tracked]++;
            chest_uncertainty[loop_chest_tracked] = uncertainty_min_chest;
        }
        else {
            //we update the track that have not been detected
            if ( chest_frequency[loop_chest_tracked] > 0 )
                chest_frequency[loop_chest_tracked]--;
            if ( chest_uncertainty[loop_chest_tracked] < uncertainty_max_chest )
                chest_uncertainty[loop_chest_tracked] += uncertainty_inc_chest;
            ROS_INFO("chest[%i]:(%f, %f) has been unassociated, frequency = %i", loop_chest_tracked, chest_tracked[loop_chest_tracked].x, chest_tracked[loop_chest_tracked].y, chest_frequency[loop_chest_tracked]);
        }

    //we suppress the track with a frequency egals to 0
    int indice = 1;
    for (int loop_chest_tracked=1; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++)
        if ( ( chest_frequency[loop_chest_tracked] ) && ( distancePoints(chest_tracked[chest], chest_tracked[loop_chest_tracked]) < 2*uncertainty_max_chest ) ) {
            chest_tracked[indice] = chest_tracked[loop_chest_tracked];
            chest_frequency[indice] = chest_frequency[loop_chest_tracked];
            chest_uncertainty[indice] = chest_uncertainty[loop_chest_tracked];
            indice++;
        }
        else
            ROS_INFO("chest[%i]: (%f, %f) is suppressed", loop_chest_tracked, chest_tracked[loop_chest_tracked].x, chest_tracked[loop_chest_tracked].y);

    nb_chests_tracked = indice;

    //we create new track with the detection not associated
    for (int loop_chest_detected=0; loop_chest_detected<nb_chests_detected; loop_chest_detected++)
        if ( !best_chest_detected[loop_chest_detected] && ( distancePoints(chest_tracked[chest], chest_detected[loop_chest_detected]) < 2*uncertainty_max_chest ) ) {
            chest_tracked[nb_chests_tracked] = chest_detected[loop_chest_detected];
            chest_frequency[nb_chests_tracked] = frequency_init;
            chest_uncertainty[nb_chests_tracked] = uncertainty_min_chest;

            ROS_INFO("chest[%i]:(%f, %f) has been created", nb_chests_tracked, chest_detected[loop_chest_detected].x, chest_detected[loop_chest_detected].y);
            nb_chests_tracked++;
        }

    //*nb_pts = 0;
    for (int loop_chest_tracked=1; loop_chest_tracked<nb_chests_tracked; loop_chest_tracked++) {
        /*geometry_msgs::Point laser_chest;
        float range = ( chest_tracked[loop_chest_tracked].x - robot_position.x ) * ( chest_tracked[loop_chest_tracked].x - robot_position.x ) +
                        ( chest_tracked[loop_chest_tracked].y - robot_position.y ) * ( chest_tracked[loop_chest_tracked].y - robot_position.y );
        range = sqrt(range);
        float angle = acos ( ( chest_tracked[loop_chest_tracked].x - robot_position.x ) / range );
        if ( chest_tracked[loop_chest_tracked].y - robot_position.y < 0 )
            angle *= -1;
        angle -= robot_orientation;

        laser_chest.x = range * cos(angle);
        laser_chest.y = range * sin(angle);
        laser_chest.z = 1.2;

        display[nb_pts] = laser_chest;*/
        display[nb_pts] = chest_tracked[loop_chest_tracked];
        colors[nb_pts].r = 0.5f;
        colors[nb_pts].g = 0.0f;
        colors[nb_pts].b = 0.5f;
        colors[nb_pts].a = 1.0;
        nb_pts++;
    }

    //populateMarkerTopic();*/

    ROS_INFO("%i chests are tracked", nb_chests_tracked);
    ROS_INFO("manage_tracks_chests done\n");

    /*ROS_INFO("enter to continue");
    getchar();*/

}//manage_tracks_chests

void estimate_person() {

    ROS_INFO("estimate_person");

    /*if ( leg_frequency[left] > leg_frequency[right] )
        leg_frequency[right] = leg_frequency[left];

    //update of frequency of left_leg and right_leg
    /*if ( leg_frequency[left] > leg_frequency[right] )
        leg_frequency[right] = leg_frequency[left];
    else
        leg_frequency[left] = leg_frequency[right];*/

    int nb_association = 0;
    if ( best_leg_tracked[left] != unassociated )
        nb_association++;
    if ( best_leg_tracked[right] != unassociated )
        nb_association++;
    if ( best_chest_tracked[chest] != unassociated )
        nb_association++;

    if ( nb_association == 1 ) {
        if ( best_leg_tracked[left] != unassociated ) {
            chest_tracked[chest].x = leg_tracked[left].x-chest_left.x;
            chest_tracked[chest].y = leg_tracked[left].y-chest_left.y;

            leg_tracked[right].x = chest_tracked[chest].x+chest_right.x;
            leg_tracked[right].y = chest_tracked[chest].y+chest_right.y;
        }
        if ( best_leg_tracked[right] != unassociated ) {
            chest_tracked[chest].x = leg_tracked[right].x-chest_right.x;
            chest_tracked[chest].y = leg_tracked[right].y-chest_right.y;

            leg_tracked[left].x = chest_tracked[chest].x+chest_left.x;
            leg_tracked[left].y = chest_tracked[chest].y+chest_left.y;
        }
        if ( best_chest_tracked[chest] != unassociated ) {
            leg_tracked[left].x = chest_tracked[chest].x+chest_left.x;
            leg_tracked[left].y = chest_tracked[chest].y+chest_left.y;

            leg_tracked[right].x = chest_tracked[chest].x+chest_right.x;
            leg_tracked[right].y = chest_tracked[chest].y+chest_right.y;
        }
    }
    else
        if ( nb_association == 2 ) {
            if ( ( best_leg_tracked[left] != unassociated ) && ( best_chest_tracked[chest] != unassociated ) ) {
                chest_left.x = leg_tracked[left].x-chest_tracked[chest].x;
                chest_left.y = leg_tracked[left].y-chest_tracked[chest].y;

                leg_tracked[right].x = chest_tracked[chest].x+chest_right.x;
                leg_tracked[right].y = chest_tracked[chest].y+chest_right.y;
            }
            if ( ( best_leg_tracked[right] != unassociated ) && ( best_chest_tracked[chest] != unassociated ) ) {
                chest_right.x = leg_tracked[right].x-chest_tracked[chest].x;
                chest_right.y = leg_tracked[right].y-chest_tracked[chest].y;

                leg_tracked[left].x = chest_tracked[chest].x+chest_left.x;
                leg_tracked[left].y = chest_tracked[chest].y+chest_left.y;
            }
            if ( ( best_leg_tracked[left] != unassociated ) && ( best_leg_tracked[right] != unassociated ) ) {
                /*geometry_msgs::Point chest1;
                chest1.x = leg_tracked[left].x-chest_left.x;
                chest1.y = leg_tracked[left].y-chest_left.y;

                geometry_msgs::Point chest2;
                chest2.x = leg_tracked[right].x-chest_right.x;
                chest2.y = leg_tracked[right].y-chest_right.y;

                chest_tracked[chest].x = ( chest1.x + chest2.x ) / 2;
                chest_tracked[chest].y = ( chest1.y + chest2.y ) / 2;*/

                chest_tracked[chest].x = ( leg_tracked[left].x + leg_tracked[right].x ) / 2;
                chest_tracked[chest].y = ( leg_tracked[left].y + leg_tracked[right].y ) / 2;

                chest_left.x = leg_tracked[left].x-chest_tracked[chest].x;
                chest_left.y = leg_tracked[left].y-chest_tracked[chest].y;

                chest_right.x = leg_tracked[right].x-chest_tracked[chest].x;
                chest_right.y = leg_tracked[right].y-chest_tracked[chest].y;
            }
        }
        else
            if ( nb_association == 3 ) {
                chest_right.x = leg_tracked[right].x-chest_tracked[chest].x;
                chest_right.y = leg_tracked[right].y-chest_tracked[chest].y;

                chest_left.x = leg_tracked[left].x-chest_tracked[chest].x;
                chest_left.y = leg_tracked[left].y-chest_tracked[chest].y;
            }

    ROS_INFO("left (%f, %f), %f, %i", leg_tracked[left].x, leg_tracked[left].y, leg_uncertainty[left], leg_frequency[left]);
    ROS_INFO("right (%f, %f), %f, %i", leg_tracked[right].x, leg_tracked[right].y, leg_uncertainty[right], leg_frequency[right]);
    ROS_INFO("chest (%f, %f), %f, %i", chest_tracked[chest].x, chest_tracked[chest].y, chest_uncertainty[chest], chest_frequency[chest]);
    //getchar();

    /*geometry_msgs::Point laser_leg;
    float range = ( leg_tracked[left].x - robot_position.x ) * ( leg_tracked[left].x - robot_position.x ) +
                    ( leg_tracked[left].y - robot_position.y ) * ( leg_tracked[left].y - robot_position.y );
    range = sqrt(range);
    float angle = acos ( ( leg_tracked[left].x - robot_position.x ) / range );
    if ( leg_tracked[left].y - robot_position.y < 0 )
        angle *= -1;
    angle -= robot_orientation;

    laser_leg.x = range * cos(angle);
    laser_leg.y = range * sin(angle);
    laser_leg.z = 0;

    display[nb_pts] = laser_leg;*/
    display[nb_pts] = leg_tracked[left];
    colors[nb_pts].r = 1.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 1.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    /*range = ( leg_tracked[right].x - robot_position.x ) * ( leg_tracked[right].x - robot_position.x ) +
                        ( leg_tracked[right].y - robot_position.y ) * ( leg_tracked[right].y - robot_position.y );
    range = sqrt(range);
    angle = acos ( ( leg_tracked[right].x - robot_position.x ) / range );
    if ( leg_tracked[right].y - robot_position.y < 0 )
        angle *= -1;
    angle -= robot_orientation;

    laser_leg.x = range * cos(angle);
    laser_leg.y = range * sin(angle);
    laser_leg.z = 0;

    display[nb_pts] = laser_leg;*/
    display[nb_pts] = leg_tracked[right];
    colors[nb_pts].r = 1.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 1.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    /*geometry_msgs::Point laser_chest;
    range = ( chest_tracked[chest].x - robot_position.x ) * ( chest_tracked[chest].x - robot_position.x ) +
                    ( chest_tracked[chest].y - robot_position.y ) * ( chest_tracked[chest].y - robot_position.y );
    range = sqrt(range);
    angle = acos ( ( chest_tracked[chest].x - robot_position.x ) / range );
    if ( chest_tracked[chest].y - robot_position.y < 0 )
        angle *= -1;
    angle -= robot_orientation;

    laser_chest.x = range * cos(angle);
    laser_chest.y = range * sin(angle);
    laser_chest.z = 1.2;

    display[nb_pts] = laser_chest;*/
    display[nb_pts] = chest_tracked[chest];
    colors[nb_pts].r = 0.0f;
    colors[nb_pts].g = 0.0f;
    colors[nb_pts].b = 1.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    if ( ( ( best_leg_tracked[right] != unassociated ) && ( best_leg_tracked[left] != unassociated ) ) ||
           ( best_chest_tracked[chest] != unassociated ) )
    {
        geometry_msgs::Point position_tracked;

        if ( best_chest_tracked[chest] != unassociated ) {
            position_tracked = chest_tracked[chest];
            ROS_INFO("chest has been detected: (%f, %f)", position_tracked.x, position_tracked.y);
        }
        else {
            position_tracked.x = ( leg_tracked[left].x + leg_tracked[right].x ) / 2;
            position_tracked.y = ( leg_tracked[left].y + leg_tracked[right].y ) / 2;
            ROS_INFO("2 legs have been detected: (%f, %f)", position_tracked.x, position_tracked.y);
        }

        //detection is green
        display[nb_pts] = position_tracked;
        colors[nb_pts].r = 0.0f;
        colors[nb_pts].g = 1.0f;
        colors[nb_pts].b = 0.0f;
        colors[nb_pts].a = 1.0;
        nb_pts++;

      /*  if ( distancePoints(previous_position_tracked, position_tracked) > 0.5 ) {
                previous_position_tracked = position_tracked;*/
		pub_person_tracker.publish(position_tracked);
        //}
    }

    ROS_INFO("chest_left: (%f, %f)", chest_left.x, chest_left.y);
    ROS_INFO("chest_right: (%f, %f)", chest_right.x, chest_right.y);
    /*display[nb_pts] = leg_tracked[left];
    colors[nb_pts].r = 1.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 0.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    display[nb_pts] = leg_tracked[right];
    colors[nb_pts].r = 1.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 0.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    display[nb_pts] = chest_tracked[chest];
    colors[nb_pts].r = 1.0f;
    colors[nb_pts].g = 1.0f;
    colors[nb_pts].b = 0.0f;
    colors[nb_pts].a = 1.0;
    nb_pts++;*/


    //pub_stamped_person_tracker.publish(stamped_person_tracked);
    ROS_INFO("estimate person done\n");
    populateMarkerTopic();
    /*ROS_INFO("press enter to continue");
    getchar();*/

    person_tracked = /*( /*( leg_frequency[left] > 0 ) && ( leg_frequency[right] > 0 ) ) ||*/ ( chest_frequency[chest] > 0 );
    if ( !person_tracked )
    {
        geometry_msgs::Point position_tracked;

        position_tracked.x = 0;
        position_tracked.y = 0;
            pub_person_tracker.publish(position_tracked);
    }

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;

    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop][0] = scan->ranges[loop];
        else
            range[loop][0] = range_max;

        //transform the scan in cartesian framewrok
        /*current_scan[loop][0].x = range[loop][0] * cos(beam_angle);
        current_scan[loop][0].y = range[loop][0] * sin(beam_angle);*/
        current_scan[loop][0].x = robot_position.x + range[loop][0] * cos(beam_angle+robot_orientation);
        current_scan[loop][0].y = robot_position.y + range[loop][0] * sin(beam_angle+robot_orientation);
        current_scan[loop][0].z = 0.0;
        //ROS_INFO("laser[%i]: (%f, %f) -> (%f, %f)", loop, range[loop], beam_angle*180/M_PI, current_scan[loop].x, current_scan[loop].y);

        laser_scan[loop][0].x = range[loop][0] * cos(beam_angle);
        laser_scan[loop][0].y = range[loop][0] * sin(beam_angle);
        laser_scan[loop][0].z = 0;
    }

}//scanCallback

void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser2 = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for (int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            range[loop][1] = scan->ranges[loop];
        else
            range[loop][1] = range_max /*+ 0.2*/;

        //transform the scan in cartesian framework
        current_scan[loop][1].x = transform_laser.x + robot_position.x + range[loop][1] * cos(beam_angle+robot_orientation);
        current_scan[loop][1].y = robot_position.y + range[loop][1] * sin(beam_angle+robot_orientation);
        /*current_scan[loop][1].x = transform_laser.x + range[loop][1] * cos(beam_angle);
        current_scan[loop][1].y = range[loop][1] * sin(beam_angle);*/
        current_scan[loop][1].z = transform_laser.z;

        laser_scan[loop][1].x = transform_laser.x + range[loop][1] * cos(beam_angle);
        laser_scan[loop][1].y = range[loop][1] * sin(beam_angle);
        laser_scan[loop][1].z = transform_laser.z;
    }

}//scanCallback2

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    previous_odom.x = current_odom.x;
    current_odom.x = o->pose.pose.position.x;

    previous_odom.y = current_odom.y;
    current_odom.y = o->pose.pose.position.y;

    previous_odom.z = current_odom.z;
    current_odom.z = tf::getYaw(o->pose.pose.orientation);

}//odomCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
    current_robot_moving = state->data;

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
    references.ns = "one_person_detector_tracker";
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

    pub_person_detector_tracker_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "one_person_detector_tracker";
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

    pub_person_detector_tracker_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "person_detector_tracker");

    ROS_INFO("waiting for activation of person detector and tracker");
    person_detector_tracker bsObject;

    ros::spin();

    return 0;
}
