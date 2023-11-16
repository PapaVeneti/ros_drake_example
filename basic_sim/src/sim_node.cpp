#include "drake/common/text_logging.h" 
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/examples/pendulum/pendulum_plant.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/JointState.h"

//maybe throw some of these
#include <fstream>
#include <string>
#include <cmath>


/** This monitor function publishes the simulation clock in the `/clock` topic. So every 
node subscribes to this topic to synchronize with the simulation clock when using
the `ros::Time` API. 
*/
drake::systems::EventStatus publishClock(const ros::Publisher & pub, const drake::systems::Context<double> & context ){    
    
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time(context.get_time());
    pub.publish(clock_msg);

    return drake::systems::EventStatus::Succeeded();
}

/**

TO DO : 
1. change xd to be an eigen vector containing the whole state
*/
class simInterface{
public:
  simInterface(const sensor_msgs::JointState & InitialJointState): n() {

    controllerSub  = n.subscribe("/joint1_position_controller", 1000, &simInterface::controllerCallback,this);
    encoderPub     = n.advertise<sensor_msgs::JointState>("/joint_states",10);

    joint_state = InitialJointState; //gets joint names
    xd = InitialJointState.position[0];
  }

  inline double get_command(void){return xd;}

  void encoderPublish(const sensor_msgs::JointState & CurrentJointState){
    joint_state = CurrentJointState;
    encoderPub.publish(joint_state);

  } 

private:

  void controllerCallback(const std_msgs::Float64::ConstPtr& command){
    ROS_INFO("[simController]: New command: %.3lf", command->data);
    xd = command->data; 
  }

  std_msgs::Float64 command;
  sensor_msgs::JointState joint_state;
  double xd;

  ros::NodeHandle n;
  ros::Subscriber controllerSub;
  ros::Publisher encoderPub;

};



int main(int argc, char **argv){

// Node Set up 
ros::init(argc, argv, "Simulator");
ros::NodeHandle n;
ros::Publisher clock_publisher = n.advertise<rosgraph_msgs::Clock>("/clock",10);

double loop_freq = 100;
ros::Rate loop_rate(loop_freq); //HZ

// system  set up
#pragma region

drake::systems::DiagramBuilder<double> builder;

drake::examples::pendulum::PendulumPlant<double>* pendulum  = 
builder.AddNamedSystem<drake::examples::pendulum::PendulumPlant<double>>("Pendulum");

drake::systems::controllers::PidController<double>* controller = 
builder.AddNamedSystem<drake::systems::controllers::PidController<double>>("Controller",
                                                                            Eigen::VectorXd::Constant(1,10),
                                                                            Eigen::VectorXd::Constant(1,1),
                                                                            Eigen::VectorXd::Constant(1,1));

builder.Connect(pendulum->get_state_output_port(),controller->get_input_port_estimated_state());
builder.Connect(controller->get_output_port_control(),pendulum->get_input_port());

builder.ExportInput(controller->get_input_port_desired_state());
builder.ExportOutput(pendulum->get_state_output_port());

auto diagram = builder.Build();
diagram -> set_name("Diagram");

// END BUILDING DIAGRAM OF SIMULATED SYSTEM
#pragma endregion

// simulation  set up
#pragma region
drake::systems::Simulator simulator(*diagram);

//Set monitoring function
auto bindedClockFun = std::bind(publishClock,clock_publisher,std::placeholders::_1);
// Create a function object
std::function<drake::systems::EventStatus(const drake::systems::Context<double> & )> monitor_func = bindedClockFun;
// Pass the monitor function to the simulator
simulator.set_monitor(monitor_func);

//simulator options
simulator.set_target_realtime_rate(1.2); // a little lower due to publishing
simulator.get_mutable_integrator().set_maximum_step_size(0.001);


#pragma endregion


// State and control
Eigen::Vector2d x0(0,0) ;
Eigen::Vector2d xd(0,0.);

auto& context = simulator.get_mutable_context();
auto& pendulum_context = diagram->GetMutableSubsystemContext(*pendulum, &context);

// Initial condition
pendulum_context.get_mutable_continuous_state_vector().SetFromVector(x0);
// Default goal
diagram->get_input_port(0).FixValue(&context,x0); // this must be changed

sensor_msgs::JointState JointState;
//from parameter server
JointState.name.push_back("joint1");
JointState.position.push_back(x0(0));
JointState.velocity.push_back(x0(1));
JointState.header.stamp = ros::Time(0);


simInterface simInterface(JointState);




//2.  Get controller data
//3.  First Publish then simulate 

double time = 0;
while (ros::ok())
{
  auto& sim_context = simulator.get_mutable_context();

  //upate cmd:
  xd(0)=simInterface.get_command();
  diagram->get_input_port(0).FixValue(&sim_context,xd); // this must be changed

  //publish encoders:
  JointState.position[0] = sim_context.get_continuous_state_vector()[0];
  JointState.velocity[0] = sim_context.get_continuous_state_vector()[1];
  JointState.header.stamp = ros::Time( sim_context.get_time());
  simInterface.encoderPublish(JointState);

  //simulate
  time +=  1./loop_freq; 
  simulator.AdvanceTo(time);

  ros::spinOnce();
  if (!loop_rate.sleep()){ROS_WARN("slept too much");} //returns false


}
  return 0;
}