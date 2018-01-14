#include "ros/ros.h"
#include "signal.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


class HelloEcho
{
public:
  HelloEcho()
  {
    //setup publishers
    echo_pub = n.advertise<std_msgs::String>("echo_world", 1000);
    status_pub = n.advertise<std_msgs::Bool>("/echo_status", 100);

    //setup subscribers
    sub = n.subscribe("hello_world", 1000, &HelloEcho::hello_callback, this);
    ros::NodeHandle nh("~");
    nh.param<std::string>("prefix", prefix, "echo");
  }

  void hello_callback(const std_msgs::String& input){
    // runs when a message is received on topic 'hello_world'
    //  re-publishes the received message on 'echo_world'
    std_msgs::String output;
    output.data = prefix + " " + input.data;
    echo_pub.publish(output);
  }

  void pub_status(const ros::TimerEvent& time){
	// publish true to the /echo_status topic every timer event
  	std_msgs::Bool msg;
	msg.data = true;
	status_pub.publish(msg);
  }

  void run(){
	// set a timer to trigger every 2 seconds and run the pub_status method
  	ros::Timer timer = n.createTimer(ros::Duration(2), &HelloEcho::pub_status, this);
	ros::spin(); // manage ROS publishing and subscribing
  }
  
private:
  // declare class variables
  ros::NodeHandle n; 
  ros::Publisher echo_pub;
  ros::Publisher status_pub;
  ros::Subscriber sub;
  std::string prefix;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "hello_echo");

  //Create an object of class SubscribeAndPublish that will take care of everything
  HelloEcho object;

  object.run();

  return 0;
}
