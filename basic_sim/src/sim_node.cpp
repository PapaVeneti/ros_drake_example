#include "drake/common/text_logging.h" 
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/examples/pendulum/pendulum_plant.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/JointState.h"

//maybe throw some of these
#include <fstream>
#include <string>
#include <cmath>


drake::systems::EventStatus publishClock(const ros::Publisher & pub, const drake::systems::Context<double> & context ){
    
    double time =  context.get_time();
    
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time(time);
    pub.publish(clock_msg);

    return drake::systems::EventStatus::Succeeded();
}


// Subscriber
void chatterCallback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  drake::log()->info("I heard: {}", msg->data.c_str()) ; 
}


int main(int argc, char **argv){

// Node Set up 
ros::init(argc, argv, "Simulator");
ros::NodeHandle n;

double loop_freq = 10;
ros::Rate loop_rate(loop_freq); //HZ

ros::Subscriber controllerSub  = n.subscribe("chatter", 1000, chatterCallback);
ros::Publisher  encoderPub     = n.advertise<sensor_msgs::JointState>("/joint_states",10);
ros::Publisher clock_publisher = n.advertise<rosgraph_msgs::Clock>("/clock",10);

// Add a "false" delay

// encoderPub.getTopic();
// while( encoderPub.getNumSubscribers()==0){
//   loop_rate.sleep();
// }

// simulation set up
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

drake::systems::VectorLogSink<double>* logger=
drake::systems::LogVectorOutput(pendulum->get_output_port(),&builder);

logger->set_name("Logger");

auto diagram = builder.Build();
diagram -> set_name("Diagram");

drake::systems::Simulator simulator(*diagram);
auto& context = simulator.get_mutable_context();
auto& pendulum_context = diagram->GetMutableSubsystemContext(*pendulum, &context);


//Set monitoring function
auto bindedClockFun = std::bind(publishClock,clock_publisher,std::placeholders::_1);

// Create a function object
std::function<drake::systems::EventStatus(const drake::systems::Context<double> & )> monitor_func = bindedClockFun;

// Pass the monitor function to the simulator
simulator.set_monitor(monitor_func);

//simulator option
simulator.set_target_realtime_rate(0.1);
simulator.get_mutable_integrator().set_maximum_step_size(0.0001);

#pragma endregion


// State and control
Eigen::Vector2d x0(0,0) ;
Eigen::Vector2d xd(M_PI_2,0.);

pendulum_context.get_mutable_continuous_state_vector().SetFromVector(x0);
diagram->get_input_port(0).FixValue(&context,xd); // this must be changed


// Encoder Publishing
sensor_msgs::JointState JointState;
JointState.name.push_back("joint1");
JointState.position.push_back(x0(0)); 
JointState.velocity.push_back(x0(1)); 
JointState.header.stamp = ros::Time(0); 

encoderPub.publish(JointState);

// helper vars
double tf,x1f,x2f;
int nSteps;
double time = 0;


//1.  Change the logger. Get info from context immidiately
//2.  Get controller data
//3.  First Publish then simulate 

while (ros::ok())
{
  time +=  1./loop_freq; 
  simulator.AdvanceTo(time);

  auto& log = logger -> FindLog(simulator.get_context()) ;   
  nSteps = simulator.get_num_steps_taken();

  JointState.position[0] = log.data()(0,nSteps);
  JointState.velocity[0] = log.data()(1,nSteps);
  JointState.header.stamp = ros::Time(log.sample_times()[nSteps]); 
  encoderPub.publish(JointState);

  ros::spinOnce(); // get data from controller
  loop_rate.sleep(); //returns false

  // Write sim data to file (deprecated)
  #pragma region 

  // std::string dataFile = "data.csv";

  // // Create an output file stream
  // std::ofstream outData(dataFile);

  // // Check if the file stream is open
  // if (outData.is_open()) {

  //     // Concacate the matrices
  //     Eigen::MatrixX3d DATA(log.sample_times().size(),3); 
  //     DATA << log.sample_times() , log.data().transpose();

  //     outData << DATA; 
      
  //     // Close the file stream
  //     outData.close();

  //     std::cout << "String successfully written to file: " << dataFile << std::endl;
  // } else {
  //     std::cerr << "Error opening the file: " << dataFile << std::endl;
  // }

  #pragma endregion

}

  return 0;
}