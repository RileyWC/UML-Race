#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bootcamp_practice/identity.h>

void scanReaction(const bootcamp_practice::identity::ConstPtr& msg){
    ROS_INFO("I RECIEVED A STRING THAT READS \"%s\" age %d!!!\n", msg->name.c_str(), msg->age);
}

int main(int argc, char** argv){
    ros::init(argc,argv, "bootcamp_practice_subscriber_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/my_test_topic_name", 1000, scanReaction);
    ros::spin();

    return 0;
}