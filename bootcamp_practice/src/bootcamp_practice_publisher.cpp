#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bootcamp_practice/identity.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "bootcamp_practice_publisher_node");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<bootcamp_practice::identity>("/my_test_topic_name", 1000);
  ros::Rate loop_rate(1);

  bootcamp_practice::identity ID;
  ID.name = "Riley";
  ID.age = 18;

  while (ros::ok()) {
    pub.publish(ID);
    ROS_INFO("Publishing");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}