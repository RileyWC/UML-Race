#include "tf/LinearMath/Vector3.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>

std::string laser_frame;
std::string nearest_wall_frame;

int main(int argc, char** argv){
    ros::init(argc, argv, "wall_tf_listener");
    ros::NodeHandle n;
    
    n.param<std::string>("laser_frame", laser_frame, "/base_laser_link");
    n.param<std::string>("nearest_wall_frame", nearest_wall_frame, "/nearest_wall");

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    tf::Vector3 vector;
    while (n.ok()){
        tf::StampedTransform trnsfrm;
        try{
             listener.lookupTransform(laser_frame, nearest_wall_frame, ros::Time(0), trnsfrm);
        }
        catch (tf::TransformException &ex) {
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
           continue;
        }
    
        vector = trnsfrm.getOrigin();
        ROS_INFO("Nearest Wall (Listened): (%f,%f,%f)", vector.getX(), vector.getY(), vector.getZ());
        rate.sleep();
    }
    return 0;
}