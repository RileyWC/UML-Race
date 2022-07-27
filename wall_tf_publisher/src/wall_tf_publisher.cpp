#include "ros/init.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/transform_datatypes.h"
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

std::string topic_name;
std::string laser_frame;
std::string nearest_wall_frame;
std::string right_nearest_wall_frame;
std::string center_nearest_wall_frame;
std::string left_nearest_wall_frame;

//Function Declarations
void tf_publish_wall_stats_at_idx(float* ranges, int index, float angle_min, float angle_increment, std::string wall_frame, char ch );
void scanReaction(const sensor_msgs::LaserScan::ConstPtr&msg);
int get_closest_index( int start_idx, int end_idx, float* ranges);

int main(int argc, char** argv){
    ros::init(argc,argv, "wall_tf_publisher");
    ros::NodeHandle n("~");
    //params
    n.param<std::string>("topic_name", topic_name, "/robot/base_scan");
    n.param<std::string>("laser_frame", laser_frame, "/base_laser_link");
    n.param<std::string>("nearest_wall_frame", nearest_wall_frame, "/nearest_wall");
    n.param<std::string>("right_nearest_wall_frame", right_nearest_wall_frame, "/right_nearest_wall");
    n.param<std::string>("center_nearest_wall_frame", center_nearest_wall_frame, "/center_nearest_wall");
    n.param<std::string>("left_nearest_wall_frame", left_nearest_wall_frame, "/left_nearest_wall");

    ros::Subscriber sub = n.subscribe(topic_name, 1000, scanReaction);
    ros::spin();
    return 0;
}

void tf_publish_wall_stats_at_idx(float* ranges, int index, float angle_min, float angle_increment, 
std_msgs::Header::_stamp_type scan_time, std::string wall_frame, char ch){
    //trig for the x & y values
    float x = ranges[index] * cos(angle_min + (angle_increment*index));
    float y = ranges[index] * sin(angle_min + (angle_increment*index));
    //tf publishing
    static tf::TransformBroadcaster tf_br;
    tf::Transform trnsfrm;
    //creating a Vector3 with values
    tf::Vector3 vec;
    vec.setX(x);
    vec.setY(y);
    vec.setZ(0);
    trnsfrm.setOrigin(vec);
    tf::Quaternion q;
    q.setRPY(0,0,0); //ignoring rotation for now
    trnsfrm.setRotation(q);
    //Switch case for custom names and messages
    tf_br.sendTransform(tf::StampedTransform(trnsfrm, scan_time,laser_frame, wall_frame));
    ROS_INFO("smallest %c is %f at index %d.", ch, ranges[index], index);
    ROS_INFO("x: %f, y: %f.\n", x, y);
}

int get_closest_index( int start_idx, int end_idx, float* ranges){
    int index_smallest = 0;
    for (int i = start_idx; i <= end_idx; i++){
        if(i == start_idx || ranges[i] < ranges[index_smallest]){
            index_smallest = i; 
        }
    }
    return index_smallest;
}

void scanReaction(const sensor_msgs::LaserScan::ConstPtr&msg){
    //time controller
    ros::Rate rate(10.0);
    //copy over the ranges data
    float ranges[180];
    for (int i = 0; i < 180; i++){
        ranges[i] = msg->ranges[i];
    }
    //get time stamp
    std_msgs::Header::_stamp_type scan_time = msg->header.stamp;
    //publish a tf of the closest wall in each sector
    tf_publish_wall_stats_at_idx(ranges, get_closest_index(0,179, ranges), msg->angle_min, msg->angle_increment, scan_time, nearest_wall_frame, 'a');
    tf_publish_wall_stats_at_idx(ranges, get_closest_index(0,59, ranges), msg->angle_min, msg->angle_increment, scan_time, right_nearest_wall_frame, 'r');
    tf_publish_wall_stats_at_idx(ranges, get_closest_index(60,119, ranges), msg->angle_min, msg->angle_increment, scan_time, center_nearest_wall_frame ,'c');
    tf_publish_wall_stats_at_idx(ranges, get_closest_index(120,179, ranges), msg->angle_min, msg->angle_increment, scan_time, left_nearest_wall_frame, 'l');
    rate.sleep();
}