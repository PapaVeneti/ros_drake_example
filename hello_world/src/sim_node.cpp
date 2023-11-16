#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drake/common/text_logging.h" 



void chatterCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  drake::log()->info("I heard: {}", msg->data.c_str()) ; 
}

int main(int argc, char **argv){

  ros::init(argc, argv, "Simulator");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1, chatterCallback);

  // Initial Implementation
  // ros::spin();

  ros::Rate loop_rate(0.5);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}