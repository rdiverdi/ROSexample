#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

class HelloEcho
{
public:
  HelloEcho()
  {
    //setup publishers
    echo_pub = n.advertise<std_msgs::String>("echo_world", 1000);
    status_pub = n.advertise<std_msgs::Bool>("echo_status", 100);

    //setup subscribers
    sub = n.subscribe("/hello_world", 1000, &HelloEcho::hello_callback, this);
  }

  void hello_callback(const std_msgs::String& input){
    std_msgs::String output;
    output.data = input.data;
    echo_pub.publish(output);
  }

  void pub_status(const ros::TimerEvent& time){
  	std_msgs::Bool msg;
	msg.data = true;
	status_pub.publish(msg);
  }

  void run(){
  	ros::Timer timer = n.createTimer(ros::Duration(2), &HelloEcho::pub_status, this);
	ros::spin();
}

private:
  ros::NodeHandle n; 
  ros::Publisher echo_pub;
  ros::Publisher status_pub;
  ros::Subscriber sub;

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