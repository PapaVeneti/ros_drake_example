#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"


class controller {
public:
    controller() : xd(0.0), n(), controller_interface(){
        controller_interface = n.subscribe("/goal_set", 1000, &controller::interfaceCallback, this);
        encoder_sub          = n.subscribe("/joint_states", 1000, &controller::encoderCallback, this);
        controller_pub       = n.advertise<std_msgs::Float64>("/joint1_position_controller", 1000,true); //latched publisher
        ROS_INFO("[Controller]: Initialized");

    }
    
    void encoderCallback(const sensor_msgs::JointStateConstPtr & JointState){
        ROS_DEBUG("[Controller]: Pendulum position is: %.3lf", JointState->position[0]);
    }

    void interfaceCallback(const std_msgs::Float64::ConstPtr & setPoint){
        xd = setPoint->data; 
        ROS_INFO("[Controller]: New controller set point: %.3lf", xd);
        publishCommand();
    }

    void publishCommand(){
        joint1_command.data = xd ; 
        controller_pub.publish(joint1_command);
    }

private:
    double xd;
    ros::NodeHandle n;
    std_msgs::Float64 joint1_command;
    ros::Subscriber controller_interface;
    ros::Subscriber encoder_sub;
    ros::Publisher controller_pub; 
};

int main(int argc, char **argv){

  ros::init(argc, argv, "Controller_placeholder");
  controller controller;
  ros::Rate loop_rate(10);
  

  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}