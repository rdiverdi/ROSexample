#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

class HelloCounter
{
public:
  HelloCounter()
  {
    //setup publishers
    count_pub = n.advertise<std_msgs::String>("message_count", 1000);

    //setup subscribers
    sub = n.subscribe("/hello_world", 1000, &HelloCounter::hello_callback, this);
    message_count = 0;
  }

  void hello_callback(const std_msgs::String& input){
    message_count++;
    message_count %= 500;
  }


  void run(){
  ros::Rate rosrate(0.5);
  std_msgs::String msg;
  char str_holder[25];
  while (ros::ok()){
    snprintf(str_holder, 25, "%d messages received", message_count);
    msg.data = str_holder;
    count_pub.publish(msg);
    ros::spinOnce();
    rosrate.sleep();
  }
}

private:
  ros::NodeHandle n; 
  ros::Publisher count_pub;
  ros::Subscriber sub;

  int message_count;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "message_counter");

  //Create an object of class SubscribeAndPublish that will take care of everything
  HelloCounter object;

  object.run();

  return 0;
}