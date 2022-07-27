#include "ros/init.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

 ros::Publisher pub;

void scanReaction(const sensor_msgs::LaserScan::ConstPtr&msg){
    // copy over the ranges array (0-179)
    float ranges[180];
    int index_largest = 0;
    for (int i = 0; i < 180; i++){
        ranges[i] = msg->ranges[i];
        //while im iterating lets get the index of the largest value
        if(i == 0 || ranges[i] > ranges[index_largest]){
            index_largest = i; 
        }
    }
    //operations for moving
     float left = ranges[125]; //125 best
     float center = ranges[90]; 
     float right = ranges[65]; //65 best
     //base twist var (only change linear x and angular z)
     geometry_msgs::Twist velocity; 
     //calculate angle from center to largest index (center is 90)
     int angle_of_turn = index_largest - 90;
     float speed;
     //check if straight is just as fine as turning
     if(center >= 4.0){
         speed = 5;
         angle_of_turn = 0;
     }
     //turn before side-swiping the wall
     if(right <= 1.2){
         ROS_INFO("Im turning left its cramped\n");
         speed = 2.8; //2.8 best
         angle_of_turn =  60; //60 best
     }
     else if(left <= 1.2){
         ROS_INFO("Im turning right its cramped\n");
         speed = 2.8; //2.8 best
         angle_of_turn = -60; //-60 best
     }
     //inform user 
     ROS_INFO("           %fm\n",center);
     ROS_INFO("%fm        %d*        %fm\n", left, angle_of_turn, right);
     ROS_INFO("             speed:%.0f%%\n\n\n", speed*20);
     //move
     velocity.linear.x = speed;
     velocity.angular.z = angle_of_turn;
     //publish results
    pub.publish(velocity); 
}

int main(int argc, char** argv){
    ros::init(argc,argv, "race_solver");
    ros::NodeHandle n;

    pub = n.advertise<geometry_msgs::Twist>("robot/cmd_vel",1000);
    ros::Subscriber sub = n.subscribe("robot/base_scan", 1000, scanReaction);
    ros::spin();
    return 0;
}